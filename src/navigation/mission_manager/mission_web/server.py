#!/usr/bin/env python3
"""Local, standard-library-only HTTP server for the mission YAML editor."""

from __future__ import annotations

import argparse
import datetime as dt
import json
import os
from pathlib import Path
import socket
import stat
import sys
import tempfile
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import unquote, urlsplit


DEFAULT_BODY_LIMIT = 2 * 1024 * 1024
YAML_SUFFIXES = {".yaml", ".yml"}
STATIC_FILES = {
    "/": ("index.html", "text/html; charset=utf-8"),
    "/index.html": ("index.html", "text/html; charset=utf-8"),
    "/app.js": ("app.js", "text/javascript; charset=utf-8"),
    "/vendor/js-yaml.min.js": (
        "vendor/js-yaml.min.js",
        "text/javascript; charset=utf-8",
    ),
}


class MissionServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(
        self,
        server_address: tuple[str, int],
        missions_dir: Path,
        web_dir: Path,
        body_limit: int = DEFAULT_BODY_LIMIT,
    ) -> None:
        super().__init__(server_address, MissionRequestHandler)
        self.missions_dir = missions_dir
        self.web_dir = web_dir
        self.body_limit = body_limit


class MissionRequestHandler(BaseHTTPRequestHandler):
    server: MissionServer

    def do_GET(self) -> None:
        path = urlsplit(self.path).path
        if path in STATIC_FILES:
            self._serve_static(*STATIC_FILES[path])
            return
        if path == "/api/missions":
            self._list_missions()
            return
        prefix = "/api/missions/"
        if path.startswith(prefix):
            self._read_mission(path[len(prefix) :])
            return
        self._send_error(HTTPStatus.NOT_FOUND, "Not found")

    def do_POST(self) -> None:
        path = urlsplit(self.path).path
        prefix = "/api/missions/"
        suffix = "/save-copy"
        if path.startswith(prefix) and path.endswith(suffix):
            encoded_name = path[len(prefix) : -len(suffix)]
            self._save_copy(encoded_name)
            return
        self._send_error(HTTPStatus.NOT_FOUND, "Not found")

    def _serve_static(self, relative_path: str, content_type: str) -> None:
        path = self.server.web_dir / relative_path
        try:
            data = path.read_bytes()
        except FileNotFoundError:
            self._send_error(HTTPStatus.NOT_FOUND, "Static file not found")
            return
        except OSError:
            self._send_error(HTTPStatus.INTERNAL_SERVER_ERROR, "Could not read static file")
            return
        self._send_bytes(HTTPStatus.OK, data, content_type)

    def _list_missions(self) -> None:
        try:
            names = sorted(
                entry.name
                for entry in self.server.missions_dir.iterdir()
                if entry.name != ".last_mission"
                and entry.suffix.lower() in YAML_SUFFIXES
                and not entry.is_symlink()
                and entry.is_file()
                and self._resolves_within_missions(entry)
            )
        except OSError:
            self._send_error(HTTPStatus.INTERNAL_SERVER_ERROR, "Could not list missions")
            return
        self._send_json(HTTPStatus.OK, {"missions": names})

    def _read_mission(self, encoded_name: str) -> None:
        path = self._mission_path(encoded_name, must_exist=True)
        if path is None:
            return
        try:
            flags = os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0)
            descriptor = os.open(path, flags)
            try:
                if not stat.S_ISREG(os.fstat(descriptor).st_mode):
                    raise OSError("mission is not a regular file")
                with os.fdopen(descriptor, "rb") as mission_file:
                    descriptor = -1
                    data = mission_file.read()
            finally:
                if descriptor >= 0:
                    os.close(descriptor)
        except OSError:
            self._send_error(HTTPStatus.BAD_REQUEST, "Could not safely read mission")
            return
        self._send_bytes(HTTPStatus.OK, data, "text/yaml; charset=utf-8")

    def _save_copy(self, encoded_name: str) -> None:
        source = self._mission_path(encoded_name, must_exist=True)
        if source is None:
            return

        body = self._read_request_body()
        if body is None:
            return
        try:
            body.decode("utf-8")
        except UnicodeDecodeError:
            self._send_error(HTTPStatus.BAD_REQUEST, "Request body must be UTF-8")
            return

        try:
            filename = self._write_timestamped_copy(source.stem, body)
        except OSError:
            self._send_error(HTTPStatus.INTERNAL_SERVER_ERROR, "Could not save mission copy")
            return
        self._send_json(HTTPStatus.CREATED, {"filename": filename})

    def _mission_path(self, encoded_name: str, must_exist: bool) -> Path | None:
        try:
            name = unquote(encoded_name, errors="strict")
        except (UnicodeDecodeError, ValueError):
            self._send_error(HTTPStatus.BAD_REQUEST, "Invalid filename encoding")
            return None

        if (
            not name
            or name in {".", ".."}
            or Path(name).is_absolute()
            or Path(name).name != name
            or "/" in name
            or "\\" in name
            or Path(name).suffix.lower() not in YAML_SUFFIXES
        ):
            self._send_error(HTTPStatus.BAD_REQUEST, "Invalid mission filename")
            return None

        candidate = self.server.missions_dir / name
        try:
            resolved = candidate.resolve(strict=must_exist)
            resolved.relative_to(self.server.missions_dir)
        except FileNotFoundError:
            self._send_error(HTTPStatus.NOT_FOUND, "Mission not found")
            return None
        except (OSError, RuntimeError, ValueError):
            self._send_error(HTTPStatus.BAD_REQUEST, "Invalid mission path")
            return None

        if candidate.is_symlink():
            self._send_error(HTTPStatus.BAD_REQUEST, "Mission symlinks are not allowed")
            return None
        if must_exist and not resolved.is_file():
            self._send_error(HTTPStatus.NOT_FOUND, "Mission not found")
            return None
        return candidate

    def _resolves_within_missions(self, path: Path) -> bool:
        try:
            path.resolve(strict=True).relative_to(self.server.missions_dir)
        except (OSError, RuntimeError, ValueError):
            return False
        return True

    def _read_request_body(self) -> bytes | None:
        value = self.headers.get("Content-Length")
        if value is None:
            self._send_error(HTTPStatus.LENGTH_REQUIRED, "Content-Length is required")
            return None
        try:
            length = int(value)
        except ValueError:
            self._send_error(HTTPStatus.BAD_REQUEST, "Invalid Content-Length")
            return None
        if length < 0:
            self._send_error(HTTPStatus.BAD_REQUEST, "Invalid Content-Length")
            return None
        if length > self.server.body_limit:
            self._send_error(HTTPStatus.REQUEST_ENTITY_TOO_LARGE, "Request body is too large")
            return None
        body = self.rfile.read(length)
        if len(body) != length:
            self._send_error(HTTPStatus.BAD_REQUEST, "Incomplete request body")
            return None
        return body

    def _write_timestamped_copy(self, stem: str, body: bytes) -> str:
        timestamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        fd, temporary_name = tempfile.mkstemp(
            prefix=".mission_web_",
            suffix=".tmp",
            dir=self.server.missions_dir,
        )
        temporary_path = Path(temporary_name)
        try:
            with os.fdopen(fd, "wb") as output:
                output.write(body)
                output.flush()
                os.fsync(output.fileno())

            counter = 1
            while True:
                collision_suffix = "" if counter == 1 else f"_{counter}"
                filename = f"{stem}_{timestamp}{collision_suffix}.yaml"
                destination = self.server.missions_dir / filename
                try:
                    # Linking publishes the fully flushed temp file atomically and,
                    # unlike replace(), can never overwrite a concurrent creation.
                    os.link(temporary_path, destination)
                    break
                except FileExistsError:
                    counter += 1

            temporary_path.unlink()
            self._fsync_missions_dir()
            return filename
        except BaseException:
            try:
                temporary_path.unlink()
            except FileNotFoundError:
                pass
            raise

    def _fsync_missions_dir(self) -> None:
        try:
            directory_fd = os.open(self.server.missions_dir, os.O_RDONLY)
        except OSError:
            return
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)

    def _send_json(self, status: HTTPStatus, payload: object) -> None:
        data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self._send_bytes(status, data, "application/json; charset=utf-8")

    def _send_error(self, status: HTTPStatus, message: str) -> None:
        self._send_json(status, {"error": message})

    def _send_bytes(self, status: HTTPStatus, data: bytes, content_type: str) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Referrer-Policy", "no-referrer")
        self.send_header("X-Frame-Options", "DENY")
        self.send_header(
            "Content-Security-Policy",
            "default-src 'self'; script-src 'self'; style-src 'self' 'unsafe-inline'; "
            "connect-src 'self'; img-src 'none'; object-src 'none'; frame-ancestors 'none'",
        )
        self.end_headers()
        self.wfile.write(data)

    def log_message(self, format_string: str, *args: object) -> None:
        sys.stderr.write(
            "%s - - [%s] %s\n"
            % (self.client_address[0], self.log_date_time_string(), format_string % args)
        )


def parse_args() -> argparse.Namespace:
    web_dir = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description="Run the local Mission YAML web editor")
    parser.add_argument(
        "--missions-dir",
        type=Path,
        default=web_dir.parent / "missions",
        help="directory containing mission YAML files",
    )
    parser.add_argument(
        "--host",
        default="0.0.0.0",
        help="address to listen on; default opens the editor to the LAN",
    )
    parser.add_argument("--port", type=int, default=8080, help="TCP port to listen on")
    return parser.parse_args()


def lan_addresses() -> list[str]:
    addresses: set[str] = set()
    try:
        hostname = socket.gethostname()
        for result in socket.getaddrinfo(hostname, None, socket.AF_INET, socket.SOCK_STREAM):
            address = result[4][0]
            if not address.startswith("127."):
                addresses.add(address)
    except OSError:
        pass

    try:
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            probe.connect(("8.8.8.8", 80))
            address = probe.getsockname()[0]
            if not address.startswith("127."):
                addresses.add(address)
        finally:
            probe.close()
    except OSError:
        pass
    return sorted(addresses)


def display_urls(host: str, port: int) -> list[str]:
    if host in {"0.0.0.0", ""}:
        urls = [f"http://127.0.0.1:{port}"]
        urls.extend(f"http://{address}:{port}" for address in lan_addresses())
        return urls
    return [f"http://{host}:{port}"]


def main() -> int:
    args = parse_args()
    web_dir = Path(__file__).resolve().parent
    try:
        missions_dir = args.missions_dir.expanduser().resolve(strict=True)
    except OSError as error:
        print(f"error: missions directory is not accessible: {error}", file=sys.stderr)
        return 1
    if not missions_dir.is_dir():
        print(f"error: missions path is not a directory: {missions_dir}", file=sys.stderr)
        return 1
    if not 0 <= args.port <= 65535:
        print("error: port must be between 0 and 65535", file=sys.stderr)
        return 1

    try:
        server = MissionServer((args.host, args.port), missions_dir, web_dir)
    except OSError as error:
        print(f"error: could not listen on {args.host}:{args.port}: {error}", file=sys.stderr)
        return 1

    host, port = server.server_address[:2]
    print(f"Mission editor serving {missions_dir}")
    print(f"Listening on {host}:{port}")
    for url in display_urls(host, port):
        print(f"Open {url}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping mission editor")
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
