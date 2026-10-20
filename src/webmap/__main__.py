"""python -m src.webmap [--port 8000] [--no-browser]"""

import argparse

from src.webmap.server import serve

ap = argparse.ArgumentParser(description="Drone simulation on a real map (Leaflet + OpenStreetMap)")
ap.add_argument("--port", type=int, default=8000)
ap.add_argument("--no-browser", action="store_true", help="do not open the browser automatically")
args = ap.parse_args()
serve(args.port, open_browser=not args.no_browser)
