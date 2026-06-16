import requests
import os

# 1. Insert your OpenRouteService API key here
API_KEY = 'eyJvcmciOiI1YjNjZTM1OTc4NTExMTAwMDFjZjYyNDgiLCJpZCI6Ijc1MTJkYjM0MDJkYjQyNDRiZjNlZDc1Y2E2NDA1ZTg4IiwiaCI6Im11cm11cjY0In0='

# 2. Define 10 LONG-DISTANCE routes for vehicle testing
# IMPORTANT: OpenRouteService requires coordinates in [Longitude, Latitude] format.
routes = [
    {"name": "Salem_to_Chengalpattu",     "coords": [[78.1460, 11.6643], [79.9757, 12.6939]]},
    {"name": "Chennai_to_Bengaluru",      "coords": [[80.2707, 13.0827], [77.5946, 12.9716]]},
    {"name": "Coimbatore_to_Madurai",     "coords": [[76.9558, 11.0168], [78.1198, 9.9252]]},
    {"name": "Trichy_to_Kanyakumari",     "coords": [[78.6928, 10.7905], [77.5385, 8.0883]]},
    {"name": "Vellore_to_Tirunelveli",    "coords": [[79.1325, 12.9165], [77.7132, 8.7139]]},
    {"name": "Pondicherry_to_Ooty",       "coords": [[79.8083, 11.9416], [76.6952, 11.4102]]},
    {"name": "Erode_to_Kanchipuram",      "coords": [[77.7172, 11.3410], [79.7036, 12.8342]]},
    {"name": "Madurai_to_Rameshwaram",    "coords": [[78.1198, 9.9252], [79.3129, 9.2876]]},
    {"name": "Tirupati_to_Chennai",       "coords": [[79.4192, 13.6288], [80.2707, 13.0827]]},
    {"name": "Hosur_to_Thanjavur",        "coords": [[77.8253, 12.7409], [79.1378, 10.7870]]}
]

def generate_gpx_route(route, index):
    # The specific ORS endpoint for driving cars that returns GPX format
    url = "https://api.openrouteservice.org/v2/directions/driving-car/gpx"

    headers = {
        'Authorization': API_KEY,
        'Content-Type': 'application/json; charset=utf-8'
    }

    # The payload sending the start and end coordinates
    payload = {
        "coordinates": route["coords"]
    }

    try:
        # Make the POST request to the API
        response = requests.post(url, json=payload, headers=headers)

        # Check if the request was successful
        if response.status_code == 200:
            filename = f"Route_{index+1:02d}_{route['name']}.gpx"

            # Save the raw text response as a GPX file
            with open(filename, 'w', encoding='utf-8') as file:
                file.write(response.text)

            print(f"Success: Generated {filename}")
        else:
            print(f"Failed to generate {route['name']}.")
            print(f"Error {response.status_code}: {response.text}")

    except Exception as e:
        print(f"An error occurred with {route['name']}: {e}")

# --- Main Execution ---
if __name__ == "__main__":
    if API_KEY == 'YOUR_OPENROUTESERVICE_API_KEY':
        print("ERROR: Please replace 'YOUR_OPENROUTESERVICE_API_KEY' with your actual key from openrouteservice.org")
    else:
        print("Generating Long-Distance GPX files...")
        for i, route_data in enumerate(routes):
            generate_gpx_route(route_data, i)
        print("Process complete! Check your folder for the generated files.")
