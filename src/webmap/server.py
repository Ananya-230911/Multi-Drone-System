"""Local web server for the map page (Python standard library only).

    python -m src.webmap            # serves http://127.0.0.1:8000 and opens the browser

Endpoints: GET /  and /static/...  (the page, Leaflet bundled), POST /api/run (a scenario
request in, the recorded flight out). It binds to 127.0.0.1 only: nothing on the network can
reach it, and it never talks to anything except the browser that opened it.
"""

from __future__ import annotations

import json
import mimetypes
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from src.webmap.runner import run
from src.webmap.scenario_builder import RequestError

STATIC = Path(__file__).resolve().parent / "static"
MAX_BODY = 200_000  # bytes


class Handler(BaseHTTPRequestHandler):
    server_version = "DroneMap/1.0"

    def log_message(self, fmt: str, *args) -> None:  # quieter console
        if "/api/" in (args[0] if args else ""):
            super().log_message(fmt, *args)

    # ------------------------------------------------------------------ GET

    def do_GET(self) -> None:
        path = self.path.split("?", 1)[0]
        if path in ("/", "/index.html"):
            path = "/static/index.html"
        if not path.startswith("/static/"):
            return self._send(404, b"not found", "text/plain")
        target = (STATIC / path[len("/static/"):]).resolve()
        if STATIC not in target.parents or not target.is_file():  # no path traversal
            return self._send(404, b"not found", "text/plain")
        ctype = mimetypes.guess_type(target.name)[0] or "application/octet-stream"
        self._send(200, target.read_bytes(), ctype, cache=not target.name.endswith(".html"))

    # ------------------------------------------------------------------ POST

    def do_POST(self) -> None:
        if self.path != "/api/run":
            return self._send(404, b"not found", "text/plain")
        try:
            n = int(self.headers.get("Content-Length", "0"))
            if not 0 < n <= MAX_BODY:
                raise RequestError("request body is empty or too large")
            req = json.loads(self.rfile.read(n))
            result = run(req)
        except RequestError as exc:
            return self._json(400, {"error": str(exc)})
        except (json.JSONDecodeError, UnicodeDecodeError):
            return self._json(400, {"error": "the request is not valid JSON"})
        except Exception as exc:  # a bug must reach the user as a message, not a hung page
            return self._json(500, {"error": f"internal error: {type(exc).__name__}: {exc}"})
        self._json(200, result)

    # ------------------------------------------------------------------ helpers

    def _json(self, code: int, obj: dict) -> None:
        self._send(code, json.dumps(obj, separators=(",", ":")).encode("utf-8"), "application/json")

    def _send(self, code: int, body: bytes, ctype: str, cache: bool = False) -> None:
        self.send_response(code)
        self.send_header("Content-Type", ctype + ("; charset=utf-8" if ctype.startswith(("text/", "application/json")) else ""))
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "max-age=3600" if cache else "no-store")
        self.end_headers()
        self.wfile.write(body)


def make_server(port: int = 8000) -> ThreadingHTTPServer:
    return ThreadingHTTPServer(("127.0.0.1", port), Handler)


def serve(port: int = 8000, open_browser: bool = True) -> None:
    srv = make_server(port)
    url = f"http://127.0.0.1:{srv.server_address[1]}/"
    print(f"Drone map running at {url}  (Ctrl+C to stop)")
    if open_browser:
        webbrowser.open(url)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        srv.server_close()
