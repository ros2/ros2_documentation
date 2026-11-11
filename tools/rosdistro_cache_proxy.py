#!/usr/bin/env python3
"""Tiny local proxy for ROS distro cache gzip files.

Why this exists:
- Browsers may not be able to fetch repo.ros2.org directly due to CORS.
- This proxy is same-origin relative to your local docs server workflow.
- It enables "proxy-first, bundled-fallback" runtime behaviour.

Endpoint:
    /api/rosdistro-cache/<distro>-cache.yaml.gz

Example:
    python tools/rosdistro_cache_proxy.py --port 9000
    # Then point conf.py setting to:
    # ros_related_packages_proxy_url = 'http://127.0.0.1:9000/api/rosdistro-cache/{distro}-cache.yaml.gz'
"""

from __future__ import annotations

import argparse
import gzip
import re
import socket
import traceback
import time
import urllib.error
import urllib.parse
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Dict, Tuple

UPSTREAM_TEMPLATE = 'https://repo.ros2.org/rosdistro_cache/{distro}-cache.yaml.gz'
DISTRO_RE = re.compile(r'^[a-z0-9][a-z0-9_-]*$', re.IGNORECASE)
PATH_RE = re.compile(r'^/api/rosdistro-cache/([a-z0-9_-]+)-cache\.yaml\.gz$', re.IGNORECASE)


class CacheStore:
    """Simple in-memory TTL cache for gzip bytes by distro."""

    def __init__(self, ttl_seconds: int) -> None:
        self._ttl = max(0, ttl_seconds)
        self._data: Dict[str, Tuple[float, bytes]] = {}

    def get(self, distro: str) -> bytes | None:
        record = self._data.get(distro)
        if record is None:
            return None
        expires_at, payload = record
        if time.time() >= expires_at:
            self._data.pop(distro, None)
            return None
        return payload

    def put(self, distro: str, payload: bytes) -> None:
        self._data[distro] = (time.time() + self._ttl, payload)


class ProxyHandler(BaseHTTPRequestHandler):
    """HTTP handler serving rosdistro cache gzip responses."""

    server_version = 'RostdistroCacheProxy/1.0'
    cache: CacheStore
    timeout_seconds: int

    def _send_cors_headers(self) -> None:
        """Allow browser fetches from local docs hosts on another port."""
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')

    def do_OPTIONS(self) -> None:  # noqa: N802 (BaseHTTPRequestHandler interface)
        self.send_response(204)
        self._send_cors_headers()
        self.end_headers()

    def do_GET(self) -> None:  # noqa: N802 (BaseHTTPRequestHandler interface)
        try:
            path = urllib.parse.urlparse(self.path).path
            match = PATH_RE.match(path)
            if not match:
                self.send_error(404, 'Unknown path')
                return

            distro = match.group(1).lower()
            if not DISTRO_RE.match(distro):
                self.send_error(400, 'Invalid distro name')
                return

            if not hasattr(self, 'cache'):
                raise RuntimeError('Proxy handler is missing cache configuration')
            if not hasattr(self, 'timeout_seconds'):
                raise RuntimeError('Proxy handler is missing timeout configuration')

            payload = self.cache.get(distro)
            if payload is None:
                try:
                    payload = self._fetch_upstream(distro)
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
        except Exception as exc:  # pragma: no cover - defensive safety net for local proxy.
            traceback.print_exc()
            self.send_error(500, f'Proxy internal error: {exc}')

    def log_message(self, fmt: str, *args) -> None:
        """Compact log format."""
        super().log_message('[proxy] ' + fmt, *args)

    def _fetch_upstream(self, distro: str) -> bytes:
        url = UPSTREAM_TEMPLATE.format(distro=distro)
        request = urllib.request.Request(url, headers={'User-Agent': 'ros2-docs-cache-proxy/1.0'})
        with urllib.request.urlopen(request, timeout=self.timeout_seconds) as response:
            payload = response.read()

        # Quick sanity check: must be valid gzip bytes.
        try:
            gzip.decompress(payload)
        except OSError as exc:
            raise ValueError('response is not valid gzip') from exc
        return payload


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
    """Parse command-line arguments and run the local cache proxy server."""
    parser = argparse.ArgumentParser(description='Local proxy for rosdistro cache gz files.')
    parser.add_argument('--host', default='127.0.0.1', help='Listen host (default: 127.0.0.1)')
    parser.add_argument('--port', type=int, default=9000, help='Listen port (default: 9000)')
    parser.add_argument(
        '--cache-ttl',
        type=int,
        default=300,
        help='In-memory cache TTL seconds (default: 300)',
    )
    parser.add_argument(
        '--upstream-timeout',
        type=int,
        default=20,
        help='Upstream timeout seconds (default: 20)',
    )
    args = parser.parse_args()

    _assert_port_free(args.host, args.port)

    cache = CacheStore(ttl_seconds=args.cache_ttl)

    class ConfiguredProxyHandler(ProxyHandler):
        """Proxy handler class with shared cache and timeout configuration."""

    ConfiguredProxyHandler.cache = cache
    ConfiguredProxyHandler.timeout_seconds = args.upstream_timeout

    server = ThreadingHTTPServer((args.host, args.port), ConfiguredProxyHandler)
    print(
        f'Proxy running on http://{args.host}:{args.port} '
        '(endpoint: /api/rosdistro-cache/<distro>-cache.yaml.gz)'
    )
    server.serve_forever()


if __name__ == '__main__':
    main()

