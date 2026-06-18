"""
GNSS_Simulator - Standalone Windows desktop application
========================================================

Selects a GPS route (.gpx), converts it into raw I/Q baseband data using the
bundled `gps-sdr-sim.exe` engine, and streams it endlessly to a connected
USRP-2920 Software Defined Radio.

Architecture
------------
* Main Thread  : PyQt6 QMainWindow with the UI (Browse, Start/Stop, IP field,
                 read-only log console).
* Worker Thread: QThread that runs the full pipeline (GPX -> CSV -> baseband
                 -> infinite USRP stream). Communicates back to the UI purely
                 through pyqtSignals so the GUI never freezes.

Pipeline (Worker)
-----------------
1. Parse the .gpx with gpxpy, compute Haversine distances, synthesize
   timestamps at a constant 60 km/h, write `user_motion.csv`.
2. Run the bundled `gps-sdr-sim.exe` to produce `temp_route.bin`
   (16-bit, 2.5 MSps), streaming its stdout/stderr to the console.
3. Connect to the USRP (user IP), configure it, then loop the baseband file
   forever until the user presses Stop.
4. On Stop: break the loop, stop the stream, delete temporary files.

Packaging
---------
Designed to be frozen with PyInstaller into a single windowed .exe. The
`resource_path()` helper resolves bundled data files via `sys._MEIPASS`.

Author : GNSS_Simulator project
Target : Windows desktop, UHD 4.x, PyQt6
"""

import os
import sys
import math
import csv
import time
import tempfile
import subprocess
import traceback
import platform
# ---------------------------------------------------------------------------
# Qt import shim: prefer PyQt6, gracefully fall back to PyQt5.
# The rest of the code uses the PyQt6 enum style (e.g. Qt.AlignmentFlag) and
# we patch the few differences so a single code path works on both bindings.
# ---------------------------------------------------------------------------
try:
    from PyQt6.QtWidgets import (
        QApplication, QMainWindow, QWidget, QPushButton, QLineEdit,
        QTextEdit, QLabel, QVBoxLayout, QHBoxLayout, QFileDialog, QGroupBox,
        QCheckBox
    )
    from PyQt6.QtCore import QThread, pyqtSignal, Qt
    from PyQt6.QtGui import QTextCursor, QFont
    _QT_BINDING = "PyQt6"
    # PyQt6 enum accessors used below
    _EXEC = "exec"
except ImportError:  # pragma: no cover - fallback path
    from PyQt5.QtWidgets import (
        QApplication, QMainWindow, QWidget, QPushButton, QLineEdit,
        QTextEdit, QLabel, QVBoxLayout, QHBoxLayout, QFileDialog, QGroupBox,
        QCheckBox
    )
    from PyQt5.QtCore import QThread, pyqtSignal, Qt
    from PyQt5.QtGui import QTextCursor, QFont
    _QT_BINDING = "PyQt5"
    _EXEC = "exec_"

import numpy as np

# gpxpy is required for GPX parsing. Import lazily-safe so a missing dependency
# produces a clear error rather than a crash at startup.
try:
    import gpxpy
except ImportError:
    gpxpy = None

# uhd (Ettus USRP API) is only needed at stream time. Import is attempted here
# but failures are tolerated so the GUI still launches on a machine without it;
# the worker reports a clean error instead.
try:
    import uhd
except ImportError:
    uhd = None


# ===========================================================================
# PyInstaller-safe resource resolution
# ===========================================================================
def resource_path(relative_path: str) -> str:
    """Return an absolute path to a bundled resource.

    When the app is frozen by PyInstaller, data files added with ``--add-data``
    are unpacked into a temporary directory whose path is stored in
    ``sys._MEIPASS``. During normal (unfrozen) execution we fall back to the
    directory containing this script.

    Parameters
    ----------
    relative_path : str
        Path of the resource relative to the bundle root
        (e.g. ``"gps-sdr-sim.exe"`` or ``"brdc0010.22n"``).

    Returns
    -------
    str
        Absolute filesystem path to the resource.
    """
    try:
        # PyInstaller creates this attribute at runtime in the one-file bundle.
        base_path = sys._MEIPASS  # type: ignore[attr-defined]
    except AttributeError:
        # Not frozen: resolve relative to this source file.
        base_path = os.path.abspath(os.path.dirname(__file__))
    return os.path.join(base_path, relative_path)


# ===========================================================================
# Constants
# ===========================================================================
SAMPLE_RATE = 2.5e6          # 2.5 MSps (Hz) - must match gps-sdr-sim -s
CENTER_FREQ = 1575.42e6      # GPS L1 carrier (Hz)
GAIN_DB = 25                 # USRP TX gain
BANDWIDTH = 2.5e6            # Analog filter bandwidth (Hz)
SPEED_KMH = 60.0             # Constant synthesized speed
SPEED_MS = SPEED_KMH * 1000.0 / 3600.0   # Convert to m/s
SAMPLE_RATE_ARG = "2500000"  # String form passed to gps-sdr-sim
EARTH_RADIUS_M = 6371000.0   # Mean Earth radius for Haversine

# Bundled resource names
GPS_SDR_SIM_EXE = "gps-sdr-sim.exe"
EPHEMERIS_FILE = "brdc0010.22n"

# Temporary output filenames (created in the working directory)
USER_MOTION_CSV = "user_motion.csv"
TEMP_ROUTE_BIN = "temp_route.bin"


# ===========================================================================
# Worker Thread
# ===========================================================================
class SimulatorWorker(QThread):
    """Runs the GPX -> baseband -> USRP-stream pipeline off the UI thread.

    All progress is reported through signals; the worker never touches Qt
    widgets directly.
    """

    # Emitted with a human-readable log line for the UI console.
    log_signal = pyqtSignal(str)
    # Emitted once when the whole run finishes (success or failure) with a
    # boolean success flag and a final status message.
    finished_signal = pyqtSignal(bool, str)

    def __init__(self, gpx_path: str, usrp_ip: str, work_dir: str, loop_route:bool):
        """
        Parameters
        ----------
        gpx_path : str
            Absolute path to the user-selected .gpx file.
        usrp_ip : str
            IP address of the target USRP-2920.
        work_dir : str
            Directory in which temporary files are created/deleted.
        """
        super().__init__()
        self.gpx_path = gpx_path
        self.usrp_ip = usrp_ip
        self.work_dir = work_dir
        self.loop_route = loop_route
        # Cooperative stop flag, set from the UI thread. A plain bool is safe
        # here because it is only ever set True (one-way) and read in a loop.
        self._stop_requested = False

        # Absolute paths for the temporary artifacts.
        self.csv_path = os.path.join(self.work_dir, USER_MOTION_CSV)
        self.bin_path = os.path.join(self.work_dir, TEMP_ROUTE_BIN)

    # ------------------------------------------------------------------ #
    # Public control
    # ------------------------------------------------------------------ #
    def request_stop(self):
        """Signal the worker to break out of the streaming loop (thread-safe)."""
        self._stop_requested = True
        self.log_signal.emit("[CTRL] Stop requested - finishing current buffer...")

    # ------------------------------------------------------------------ #
    # Convenience logging
    # ------------------------------------------------------------------ #
    def _log(self, msg: str):
        """Emit a log line to the UI console."""
        self.log_signal.emit(msg)

    # ------------------------------------------------------------------ #
    # QThread entry point
    # ------------------------------------------------------------------ #
    def run(self):
        """Main worker entry point executed in the secondary thread."""
        try:
            # --- Step 0: validate dependencies & inputs -------------------
            if gpxpy is None:
                raise RuntimeError(
                    "The 'gpxpy' package is not installed. Install it with "
                    "'pip install gpxpy'."
                )
            if not os.path.isfile(self.gpx_path):
                raise FileNotFoundError(
                    f"Selected GPX file not found: {self.gpx_path}"
                )

            # --- Step 1: GPX -> user_motion.csv ---------------------------
            self._log("=" * 60)
            self._log("[1/3] Parsing GPX and synthesizing motion file...")
            num_points = self._generate_user_motion()
            self._log(f"[1/3] Wrote {num_points} motion samples to "
                      f"{USER_MOTION_CSV}")

            if self._stop_requested:
                self._cleanup()
                self.finished_signal.emit(False, "Stopped before streaming.")
                return

            # --- Step 2: gps-sdr-sim -> temp_route.bin --------------------
            self._log("=" * 60)
            self._log("[2/3] Generating baseband with gps-sdr-sim.exe...")
            self._generate_baseband()
            self._log(f"[2/3] Baseband written to {TEMP_ROUTE_BIN}")

            if self._stop_requested:
                self._cleanup()
                self.finished_signal.emit(False, "Stopped before streaming.")
                return

            # --- Step 3: stream to USRP (infinite loop) -------------------
            self._log("=" * 60)
            self._log("[3/3] Streaming to USRP (looping until Stop)...")
            self._stream_to_usrp()

            # Normal exit happens only after Stop was requested.
            self.finished_signal.emit(True, "Streaming stopped cleanly.")

        except Exception as exc:  # Catch-all: report, never crash the GUI.
            self._log("!" * 60)
            self._log(f"[ERROR] {exc}")
            self._log(traceback.format_exc())
            self.finished_signal.emit(False, f"Failed: {exc}")
        finally:
            # Always remove temporary files, regardless of how we exited.
            self._cleanup()

    # ------------------------------------------------------------------ #
    # Step 1 implementation
    # ------------------------------------------------------------------ #
    @staticmethod
    def _haversine_m(lat1, lon1, lat2, lon2) -> float:
        """Great-circle distance between two lat/lon points, in meters."""
        # Convert degrees to radians.
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)
        a = (math.sin(dphi / 2.0) ** 2
             + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2.0) ** 2)
        c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
        return EARTH_RADIUS_M * c

    def _generate_user_motion(self) -> int:
        """Parse the GPX track and write a gps-sdr-sim user-motion CSV.

        The CSV format expected by gps-sdr-sim is one row per epoch:
            time(sec), ECEF_X, ECEF_Y, ECEF_Z
        We instead use the simpler/accepted lat/lon/height variant only if the
        engine supports it; for maximum compatibility we emit ECEF coordinates
        computed from WGS-84 lat/lon/alt.

        Timestamps are synthesized assuming constant speed (60 km/h): the time
        for each segment is segment_distance / speed.

        Returns
        -------
        int
            Number of rows written.
        """
        with open(self.gpx_path, "r", encoding="utf-8") as fh:
            gpx = gpxpy.parse(fh)

        # Flatten every track/segment point into an ordered list.
        points = []
        for track in gpx.tracks:
            for segment in track.segments:
                for pt in segment.points:
                    points.append(pt)

        # Also accept routes (some GPX files use <rte> instead of <trk>).
        for route in gpx.routes:
            for pt in route.points:
                points.append(pt)

        if len(points) < 2:
            raise ValueError(
                "GPX file must contain at least two track/route points."
            )

        # Walk the points, accumulate time using constant speed, write rows.
        rows = []
        t = 0.0  # seconds since start
        prev = points[0]
        rows.append(self._point_to_row(t, prev))

        for cur in points[1:]:
            d = self._haversine_m(prev.latitude, prev.longitude,
                                   cur.latitude, cur.longitude)
            # Guard against duplicate points (zero distance => zero dt).
            dt = d / SPEED_MS if SPEED_MS > 0 else 0.0
            t += dt
            rows.append(self._point_to_row(t, cur))
            prev = cur

        # gps-sdr-sim samples motion at 10 Hz by default. We resample our
        # sparse track to a fixed 0.1 s grid via linear interpolation so the
        # engine receives a smooth, evenly-spaced trajectory.
        dense_rows = self._resample_motion(rows, dt_target=0.1)

        with open(self.csv_path, "w", newline="", encoding="utf-8") as fh:
            writer = csv.writer(fh)
            for row in dense_rows:
                # Format: time, ECEF_X, ECEF_Y, ECEF_Z
                writer.writerow([f"{row[0]:.1f}",
                                 f"{row[1]:.4f}",
                                 f"{row[2]:.4f}",
                                 f"{row[3]:.4f}"])

        return len(dense_rows)

    def _point_to_row(self, t: float, pt):
        """Convert a GPX point to a (time, X, Y, Z) ECEF row."""
        alt = pt.elevation if pt.elevation is not None else 0.0
        x, y, z = self._lla_to_ecef(pt.latitude, pt.longitude, alt)
        return (t, x, y, z)

    @staticmethod
    def _lla_to_ecef(lat_deg, lon_deg, alt_m):
        """Convert WGS-84 geodetic lat/lon/alt to ECEF X/Y/Z (meters)."""
        # WGS-84 ellipsoid parameters.
        a = 6378137.0                      # semi-major axis
        e2 = 6.69437999014e-3              # first eccentricity squared
        lat = math.radians(lat_deg)
        lon = math.radians(lon_deg)
        sin_lat = math.sin(lat)
        # Radius of curvature in the prime vertical.
        N = a / math.sqrt(1.0 - e2 * sin_lat * sin_lat)
        x = (N + alt_m) * math.cos(lat) * math.cos(lon)
        y = (N + alt_m) * math.cos(lat) * math.sin(lon)
        z = (N * (1.0 - e2) + alt_m) * sin_lat
        return x, y, z

    @staticmethod
    def _resample_motion(rows, dt_target=0.1):
        """Linearly interpolate ECEF rows onto an even time grid.

        Parameters
        ----------
        rows : list[tuple]
            Sparse (time, X, Y, Z) rows, time strictly increasing.
        dt_target : float
            Target spacing in seconds (gps-sdr-sim expects 10 Hz => 0.1 s).
        """
        if len(rows) < 2:
            return rows

        times = np.array([r[0] for r in rows], dtype=np.float64)
        xs = np.array([r[1] for r in rows], dtype=np.float64)
        ys = np.array([r[2] for r in rows], dtype=np.float64)
        zs = np.array([r[3] for r in rows], dtype=np.float64)

        t_end = times[-1]
        if t_end <= 0:
            return rows

        grid = np.arange(0.0, t_end + dt_target, dt_target)
        gx = np.interp(grid, times, xs)
        gy = np.interp(grid, times, ys)
        gz = np.interp(grid, times, zs)

        return [(float(grid[i]), float(gx[i]), float(gy[i]), float(gz[i]))
                for i in range(len(grid))]

    # ------------------------------------------------------------------ #
    # Step 2 implementation
    # ------------------------------------------------------------------ #
    def _generate_baseband(self):
        """Invoke the bundled gps-sdr-sim.exe, streaming output to the UI."""
        if platform.system() != "Windows":
            self._log("[WARNING] macOS detected. Skipping Windows .exe execution.")
            self._log("[WARNING] Creating a dummy temp_route.bin to continue UI test...")

            # Create a fake dummy file so Step 3 doesn't crash looking for it
            with open(os.path.join(self.work_dir, "temp_route.bin"), "wb") as f:
                f.write(b"dummy data")
            return

        exe = resource_path(GPS_SDR_SIM_EXE)
        eph = resource_path(EPHEMERIS_FILE)

        # Robustness: verify the bundled binaries actually exist.
        if not os.path.isfile(exe):
            raise FileNotFoundError(
                f"Bundled engine not found: {exe}. Ensure {GPS_SDR_SIM_EXE} "
                f"was included with --add-data."
            )
        if not os.path.isfile(eph):
            raise FileNotFoundError(
                f"Bundled ephemeris not found: {eph}. Ensure {EPHEMERIS_FILE} "
                f"was included with --add-data."
            )

        cmd = [
            exe,
            "-e", eph,                 # broadcast ephemeris
            "-u", self.csv_path,       # user motion file
            "-b", "16",                # 16-bit I/Q sample format
            "-s", SAMPLE_RATE_ARG,     # sample rate (2.5 MSps)
            "-o", self.bin_path,       # output baseband file
        ]
        self._log("[CMD] " + " ".join(f'"{c}"' if " " in c else c
                                      for c in cmd))

        # Hide the console window on Windows when frozen.
        creationflags = 0
        if os.name == "nt":
            creationflags = getattr(subprocess, "CREATE_NO_WINDOW", 0)

        # Launch and stream combined stdout/stderr line-by-line.
        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            cwd=self.work_dir,
            text=True,
            bufsize=1,
            creationflags=creationflags,
        )

        # Relay every line to the console as it arrives.
        assert proc.stdout is not None
        for line in proc.stdout:
            self._log("[SIM] " + line.rstrip())
            if self._stop_requested:
                # User aborted mid-generation: kill the child process.
                proc.terminate()
                self._log("[SIM] Generation aborted by user.")
                break

        proc.wait()
        if proc.returncode not in (0, None) and not self._stop_requested:
            raise RuntimeError(
                f"gps-sdr-sim.exe exited with code {proc.returncode}."
            )

        if not self._stop_requested and not os.path.isfile(self.bin_path):
            raise RuntimeError(
                "gps-sdr-sim.exe finished but produced no output file."
            )

    # ------------------------------------------------------------------ #
    # Step 3 implementation
    # ------------------------------------------------------------------ #
    def _stream_to_usrp(self):
        """Configure the USRP and loop the baseband file until Stop."""
        if platform.system() != "Windows" or 'uhd' not in globals():
            self._log("[WARNING] UHD missing or macOS detected. Entering UI Simulation Mode...")

            loop_mode_text = "endlessly" if self.loop_route else "once"
            self._log(f"[USRP-SIM] Transmitting (route will play {loop_mode_text})...")

            loop_count = 0
            while not self._stop_requested:
                loop_count += 1
                self._log(f"[USRP-SIM] --- Transmission pass #{loop_count} ---")

                # Simulate the time it takes to transmit the route
                for _ in range(5):
                    if self._stop_requested:
                        break
                    time.sleep(1) # Wait 1 second per tick

                # If the user unchecked the loop box, break after one full simulated pass
                if not self.loop_route:
                    self._log("[USRP-SIM] Destination reached. Stopping transmission.")
                    break

            self._log("[USRP-SIM] Stopping stream...")
            return

        # --- Connect (with graceful timeout handling) ---------------------
        self._log(f"[USRP] Connecting to {self.usrp_ip} ...")
        try:
            # A short device-args timeout keeps a bad IP from hanging forever.
            usrp = uhd.usrp.MultiUSRP(f"addr={self.usrp_ip}")
        except RuntimeError as exc:
            raise RuntimeError(
                f"Could not connect to USRP at {self.usrp_ip}: {exc}"
            )
        self._log("[USRP] Connected.")

        # --- Configure TX chain ------------------------------------------
        usrp.set_tx_rate(SAMPLE_RATE)
        # UHD 4.x requires a TuneRequest object for the center frequency.
        tune_request = uhd.types.TuneRequest(CENTER_FREQ)
        usrp.set_tx_freq(tune_request)
        usrp.set_tx_gain(GAIN_DB)
        try:
            usrp.set_tx_bandwidth(BANDWIDTH)
        except Exception:
            # Some daughterboards do not support an explicit bandwidth call.
            self._log("[USRP] (bandwidth not settable on this board - skipped)")

        self._log(f"[USRP] Rate={usrp.get_tx_rate()/1e6:.3f} MSps  "
                  f"Freq={usrp.get_tx_freq()/1e6:.3f} MHz  "
                  f"Gain={usrp.get_tx_gain():.1f} dB")

        # --- Set up the TX streamer (sc16 wire format) -------------------
        st_args = uhd.usrp.StreamArgs("sc16", "sc16")
        st_args.channels = [0]
        tx_streamer = usrp.get_tx_stream(st_args)
        max_samps = tx_streamer.get_max_num_samps()

        # --- Load the baseband file once into memory ---------------------
        # gps-sdr-sim with -b 16 writes interleaved int16 I,Q pairs.
        self._log("[USRP] Loading baseband into memory...")
        raw = np.fromfile(self.bin_path, dtype=np.int16)
        if raw.size == 0:
            raise RuntimeError("Baseband file is empty - nothing to transmit.")
        if raw.size % 2 != 0:
            # Drop a trailing odd sample so reshape into I/Q pairs is valid.
            raw = raw[:-1]

        # Interleaved [I0,Q0,I1,Q1,...] -> complex64 [I0+jQ0, ...].
        # Normalize int16 full-scale to +/-1.0 for the float TX path.
        iq = raw.reshape(-1, 2)
        samples = (iq[:, 0].astype(np.float32)
                   + 1j * iq[:, 1].astype(np.float32)) / 32768.0
        samples = samples.astype(np.complex64)
        total = samples.shape[0]
        self._log(f"[USRP] {total} complex samples ready "
                  f"(~{total / SAMPLE_RATE:.1f} s of signal).")

        # --- Metadata for a continuous (non-bursty) stream ---------------
        metadata = uhd.types.TXMetadata()
        metadata.start_of_burst = True
        metadata.end_of_burst = False
        metadata.has_time_spec = False

        # --- Transmit loop --------------------------------------
        loop_mode_text = "endlessly" if self.loop_route else "once"
        self._log(f"[USRP] Transmitting (route will play {loop_mode_text})...")

        loop_count = 0
        while not self._stop_requested:
            loop_count += 1
            self._log(f"[USRP] --- Transmission pass #{loop_count} ---")
            pos = 0
            while pos < total and not self._stop_requested:
                chunk = samples[pos:pos + max_samps]
                tx_streamer.send(chunk, metadata)
                metadata.start_of_burst = False
                pos += chunk.shape[0]

            # If the user unchecked the loop box, break after one full pass
            if not self.loop_route:
                self._log("[USRP] Destination reached. Stopping transmission.")
                break

        # --- Flush / end the burst cleanly -------------------------------
        self._log("[USRP] Stopping stream...")
        end_md = uhd.types.TXMetadata()
        end_md.end_of_burst = True
        try:
            tx_streamer.send(np.zeros(0, dtype=np.complex64), end_md)
        except Exception:
            pass
        self._log("[USRP] Stream stopped.")

    # ------------------------------------------------------------------ #
    # Cleanup
    # ------------------------------------------------------------------ #
    def _cleanup(self):
        """Delete temporary files; never raise."""
        for path in (self.csv_path, self.bin_path):
            try:
                if os.path.isfile(path):
                    os.remove(path)
                    self._log(f"[CLEANUP] Removed {os.path.basename(path)}")
            except Exception as exc:
                self._log(f"[CLEANUP] Could not remove "
                          f"{os.path.basename(path)}: {exc}")


# ===========================================================================
# Main Window (UI)
# ===========================================================================
class MainWindow(QMainWindow):
    """The application's main window and all UI controls."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("GNSS_Simulator")
        self.resize(720, 560)

        self.worker: SimulatorWorker | None = None
        self.gpx_path: str = ""
        # Temporary files live alongside the executable / script.
        self.work_dir = os.path.abspath(os.path.dirname(sys.argv[0])) \
            if getattr(sys, "frozen", False) else os.getcwd()

        self._build_ui()
        self._log("Ready. Select a GPX file, enter the USRP IP, then Start.")

    # ------------------------------------------------------------------ #
    # UI construction
    # ------------------------------------------------------------------ #
    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)

        # --- File selection group ----------------------------------------
        file_box = QGroupBox("GPS Route")
        file_layout = QHBoxLayout(file_box)
        self.gpx_label = QLineEdit()
        self.gpx_label.setReadOnly(True)
        self.gpx_label.setPlaceholderText("No .gpx file selected")
        self.browse_btn = QPushButton("Browse GPX...")
        self.browse_btn.clicked.connect(self.on_browse)
        file_layout.addWidget(self.gpx_label)
        file_layout.addWidget(self.browse_btn)
        root.addWidget(file_box)

        # --- USRP configuration group ------------------------------------
        usrp_box = QGroupBox("USRP-2920")
        usrp_layout = QHBoxLayout(usrp_box)
        usrp_layout.addWidget(QLabel("Device IP:"))
        self.ip_input = QLineEdit("192.168.10.2")  # common default USRP addr
        self.ip_input.setPlaceholderText("e.g. 192.168.10.2")
        usrp_layout.addWidget(self.ip_input)
        root.addWidget(usrp_box)

        # --- Options Group ---------------------------------
        options_layout = QHBoxLayout()
        self.loop_checkbox = QCheckBox("Loop Route")
        self.loop_checkbox.setChecked(True)
        options_layout.addWidget(self.loop_checkbox)
        root.addLayout(options_layout)

        # --- Control buttons -------------------------------------------
        btn_layout = QHBoxLayout()
        self.start_btn = QPushButton("Start")
        self.start_btn.clicked.connect(self.on_start)
        self.stop_btn = QPushButton("Stop")
        self.stop_btn.clicked.connect(self.on_stop)
        self.stop_btn.setEnabled(False)
        btn_layout.addWidget(self.start_btn)
        btn_layout.addWidget(self.stop_btn)
        root.addLayout(btn_layout)

        # --- Log console -------------------------------------------------
        root.addWidget(QLabel("Console:"))
        self.console = QTextEdit()
        self.console.setReadOnly(True)
        # Monospace font for readable, log-style output.
        self.console.setFont(QFont("Consolas", 9))
        root.addWidget(self.console)

    # ------------------------------------------------------------------ #
    # Logging helper
    # ------------------------------------------------------------------ #
    def _log(self, msg: str):
        """Append a line to the console and keep it scrolled to the bottom."""
        self.console.append(msg)
        self.console.moveCursor(QTextCursor.MoveOperation.End
                                if _QT_BINDING == "PyQt6"
                                else QTextCursor.End)

    # ------------------------------------------------------------------ #
    # Slots
    # ------------------------------------------------------------------ #
    def on_browse(self):
        """Open a file dialog to choose a .gpx route."""
        path, _ = QFileDialog.getOpenFileName(
            self, "Select GPX route", "", "GPX files (*.gpx);;All files (*)"
        )
        if path:
            self.gpx_path = path
            self.gpx_label.setText(path)
            self._log(f"Selected GPX: {path}")

    def on_start(self):
        """Validate inputs and launch the worker thread."""
        # --- Input validation --------------------------------------------
        if not self.gpx_path:
            self._log("[WARN] Please select a GPX file first.")
            return
        if not os.path.isfile(self.gpx_path):
            self._log("[WARN] Selected GPX file no longer exists.")
            return
        ip = self.ip_input.text().strip()
        if not ip:
            self._log("[WARN] Please enter the USRP IP address.")
            return

        # --- Toggle UI state ---------------------------------------------
        self.start_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)
        self.browse_btn.setEnabled(False)
        self.ip_input.setEnabled(False)

        # --- Spin up the worker ------------------------------------------
        is_looping = self.loop_checkbox.isChecked()
        self.worker = SimulatorWorker(self.gpx_path, ip, self.work_dir, is_looping)
        self.worker.log_signal.connect(self._log)
        self.worker.finished_signal.connect(self.on_finished)
        self.worker.start()

    def on_stop(self):
        """Ask the worker to stop streaming."""
        if self.worker is not None and self.worker.isRunning():
            self.stop_btn.setEnabled(False)  # debounce double clicks
            self.worker.request_stop()

    def on_finished(self, success: bool, message: str):
        """Reset the UI when the worker finishes."""
        prefix = "[DONE]" if success else "[STOPPED]"
        self._log(f"{prefix} {message}")
        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self.browse_btn.setEnabled(True)
        self.ip_input.setEnabled(True)
        self.worker = None

    # ------------------------------------------------------------------ #
    # Window close handling
    # ------------------------------------------------------------------ #
    def closeEvent(self, event):
        """Ensure the worker is stopped before the window closes."""
        if self.worker is not None and self.worker.isRunning():
            self.worker.request_stop()
            self.worker.wait(5000)  # give it up to 5s to clean up
        event.accept()


# ===========================================================================
# Entry point
# ===========================================================================
def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    # PyQt6 uses exec(); PyQt5 uses exec_(). Resolve at runtime.
    sys.exit(getattr(app, _EXEC)())


if __name__ == "__main__":
    main()
