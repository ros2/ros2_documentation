#!/usr/bin/env python3
"""Serve built Sphinx HTML and rosdistro cache API on one origin (local testing).

Use this instead of ``python -m http.server`` when testing proxy-first related
packages. The browser can fetch ``/api/rosdistro-cache/<distro>-cache.yaml.gz``
same-origin (no cross-port CORS quirks).

Build docs (proxy URL is the conf.py default; env override optional)::

    make html

Then run::

    python tools/serve_docs_with_proxy.py

Open http://127.0.0.1:8000/... and check DevTools Network for a 200 on
``/api/rosdistro-cache/rolling-cache.yaml.gz``.
"""

from __future__ import annotations

import argparse
import gzip
import re
import socket
import urllib.error
import urllib.parse
import urllib.request
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

UPSTREAM_TEMPLATE = 'https://repo.ros2.org/rosdistro_cache/{distro}-cache.yaml.gz'
DISTRO_RE = re.compile(r'^[a-z0-9][a-z0-9_-]*$', re.IGNORECASE)
PATH_RE = re.compile(r'^/api/rosdistro-cache/([a-z0-9_-]+)-cache\.yaml\.gz$', re.IGNORECASE)


class CacheStore:
    """Simple in-memory TTL cache for gzip bytes by distro."""

    def __init__(self, ttl_seconds: int) -> None:
        import time

        self._time = time
        self._ttl = max(0, ttl_seconds)
        self._data: dict[str, tuple[float, bytes]] = {}

    def get(self, distro: str) -> bytes | None:
        record = self._data.get(distro)
        if record is None:
            return None
        expires_at, payload = record
        if self._time.time() >= expires_at:
            self._data.pop(distro, None)
            return None
        return payload

    def put(self, distro: str, payload: bytes) -> None:
        self._data[distro] = (self._time.time() + self._ttl, payload)


def _fetch_upstream(distro: str, timeout_seconds: int) -> bytes:
    url = UPSTREAM_TEMPLATE.format(distro=distro)
    request = urllib.request.Request(url, headers={'User-Agent': 'ros2-docs-cache-proxy/1.0'})
    with urllib.request.urlopen(request, timeout=timeout_seconds) as response:
        payload = response.read()
    try:
        gzip.decompress(payload)
    except OSError as exc:
        raise ValueError('response is not valid gzip') from exc
    return payload


class DocsWithProxyHandler(SimpleHTTPRequestHandler):
    """Static files from *directory*; ``/api/rosdistro-cache/...`` proxied upstream."""

    cache: CacheStore
    upstream_timeout: int

    def _send_cors_headers(self) -> None:
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')

    def do_OPTIONS(self) -> None:  # noqa: N802
        path = urllib.parse.urlparse(self.path).path
        if PATH_RE.match(path):
            self.send_response(204)
            self._send_cors_headers()
            self.end_headers()
            return
        super().do_OPTIONS()

    def do_GET(self) -> None:  # noqa: N802
        path = urllib.parse.urlparse(self.path).path
        match = PATH_RE.match(path)
        if not match:
            return super().do_GET()

        distro = match.group(1).lower()
        if not DISTRO_RE.match(distro):
            self.send_error(400, 'Invalid distro name')
            return

        payload = self.cache.get(distro)
        if payload is None:
            try:
                payload = _fetch_upstream(distro, self.upstream_timeout)
            except urllib.error.HTTPError as exc:
                self.send_error(exc.code, f'Upstream HTTP error: {exc.reason}')
                return
            except urllib.error.URLError as exc:
                self.send_error(502, f'Upstream URL error: {exc.reason}')
                return
            except TimeoutError:
                self.send_error(504, 'Upstream timeout')
                return
            except ValueError as exc:
                self.send_error(502, f'Bad upstream payload: {exc}')
                return
            self.cache.put(distro, payload)

        self.send_response(200)
        self._send_cors_headers()
        self.send_header('Content-Type', 'application/gzip')
        self.send_header('Cache-Control', 'public, max-age=300')
        self.send_header('Content-Length', str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def log_message(self, fmt: str, *args) -> None:
        super().log_message('[docs+proxy] ' + fmt, *args)


def _assert_port_free(host: str, port: int) -> None:
    """Fail fast when another local server already owns the port (common on Windows)."""
    probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        if hasattr(socket, 'SO_EXCLUSIVEADDRUSE'):
            probe.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        probe.bind((host, port))
    except OSError as exc:
        raise SystemExit(
            f'Port {port} on {host} is already in use ({exc}).\n'
            'Stop leftover python/http.server processes or pass --port with a free value.'
        ) from exc
    finally:
        probe.close()


def main() -> None:
    """Parse command-line arguments and serve built HTML plus the cache API."""
    repo = Path(__file__).resolve().parents[1]
    default_html = repo / 'build' / 'html'

    parser = argparse.ArgumentParser(
        description='Serve build/html and /api/rosdistro-cache/ on one port.',
    )
    parser.add_argument('--host', default='127.0.0.1', help='Listen host (default: 127.0.0.1)')
    parser.add_argument('--port', type=int, default=8000, help='Listen port (default: 8000)')
    parser.add_argument(
        '--directory',
        type=Path,
        default=default_html,
        help=f'HTML root (default: {default_html})',
    )
    parser.add_argument('--cache-ttl', type=int, default=300, help='Proxy cache TTL seconds')
    parser.add_argument('--upstream-timeout', type=int, default=20, help='Upstream timeout seconds')
    args = parser.parse_args()

    html_dir = args.directory.resolve()
    if not html_dir.is_dir():
        raise SystemExit(f'HTML directory not found: {html_dir}\nRun make html first.')

    _assert_port_free(args.host, args.port)

    cache_store = CacheStore(ttl_seconds=args.cache_ttl)
    html_dir_str = str(html_dir)

    class ConfiguredHandler(DocsWithProxyHandler):
        """Handler with shared cache and HTML root."""

        def __init__(self, request, client_address, server):
            super().__init__(request, client_address, server, directory=html_dir_str)

    ConfiguredHandler.cache = cache_store
    ConfiguredHandler.upstream_timeout = args.upstream_timeout

    server = ThreadingHTTPServer((args.host, args.port), ConfiguredHandler)
    print(f'Serving {html_dir}')
    print(f'Open http://{args.host}:{args.port}/')
    print('API: /api/rosdistro-cache/<distro>-cache.yaml.gz')
    print(
        'Proxy URL default is in conf.py; override with ROS_RELATED_PACKAGES_PROXY_URL if needed'
    )
    server.serve_forever()


if __name__ == '__main__':
    main()
