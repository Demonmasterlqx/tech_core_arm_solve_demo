#!/usr/bin/env python3
"""Fetch byte-verified official RMUC CAD, preserving resumable partial files."""

from __future__ import annotations

import argparse
import hashlib
import http.client
import json
import os
from pathlib import Path
import re
import sys
import time
import urllib.error
import urllib.request


ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / "reference" / "cad_sources.json"
CHUNK_SIZE = 1024 * 1024
TIMEOUT_SECONDS = 30
ATTEMPTS = 3


class VerificationError(Exception):
    """An existing file or response contradicts the pinned asset metadata."""


def verify(path: Path, asset: dict) -> None:
    expected = asset["size_bytes"]
    size = path.stat().st_size
    if size != expected:
        raise VerificationError(f"{path}: expected {expected} bytes, found {size}")
    md5 = hashlib.md5()
    sha256 = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(CHUNK_SIZE), b""):
            md5.update(chunk)
            sha256.update(chunk)
    if md5.hexdigest() != asset["official_md5"]:
        raise VerificationError(f"{path}: official MD5 mismatch; file retained")
    if sha256.hexdigest() != asset["sha256"]:
        raise VerificationError(f"{path}: SHA-256 mismatch; file retained")


def transfer(part: Path, asset: dict) -> None:
    """Append one validated HTTP response; interrupted bytes remain resumable."""
    expected = asset["size_bytes"]
    offset = part.stat().st_size if part.exists() else 0
    if offset > expected:
        raise VerificationError(f"{part}: oversized partial file; file retained")
    headers = {
        "User-Agent": "tech-core-arm-solve-cad-fetch/1.0",
        "Accept-Encoding": "identity",
    }
    if offset:
        headers["Range"] = f"bytes={offset}-"
    request = urllib.request.Request(asset["url"], headers=headers)
    with urllib.request.urlopen(request, timeout=TIMEOUT_SECONDS) as response:
        status = response.status
        encoding = response.headers.get("Content-Encoding", "identity").lower()
        if encoding not in ("identity", ""):
            raise VerificationError(f"unexpected Content-Encoding: {encoding}")
        if status == 206:
            match = re.fullmatch(
                r"bytes (\d+)-(\d+)/(\d+)",
                response.headers.get("Content-Range", ""),
            )
            if not match:
                raise VerificationError("missing or invalid Content-Range")
            start, end, total = map(int, match.groups())
            if start != offset or total != expected or end != expected - 1:
                raise VerificationError(
                    f"inconsistent Content-Range: {response.headers['Content-Range']}"
                )
            response_bytes = end - start + 1
        elif status == 200 and offset == 0:
            response_bytes = expected
        elif status == 200:
            raise VerificationError(
                "server ignored Range; partial file retained without overwriting"
            )
        else:
            raise VerificationError(f"unexpected HTTP status: {status}")
        length = response.headers.get("Content-Length")
        if length is None or not length.isdigit() or int(length) != response_bytes:
            raise VerificationError(
                f"Content-Length {length!r} does not match {response_bytes} bytes"
            )
        received = 0
        last_report = time.monotonic()
        with part.open("ab") as target:
            while True:
                chunk = response.read(min(CHUNK_SIZE, response_bytes - received + 1))
                if not chunk:
                    break
                if received + len(chunk) > response_bytes:
                    raise VerificationError("HTTP body exceeds declared asset size")
                target.write(chunk)
                received += len(chunk)
                if time.monotonic() - last_report >= 10:
                    print(f"{asset['filename']}: {offset + received}/{expected} bytes", flush=True)
                    last_report = time.monotonic()
            target.flush()
            os.fsync(target.fileno())
        if received != response_bytes:
            raise OSError(f"short HTTP body: {received}/{response_bytes} bytes")


def fetch(output_dir: Path, asset: dict) -> None:
    final = output_dir / asset["filename"]
    part = final.with_name(final.name + ".part")
    if final.exists():
        verify(final, asset)
        print(f"Verified existing {final} ({asset['size_bytes']} bytes); no download")
        return
    expected = asset["size_bytes"]
    for attempt in range(1, ATTEMPTS + 1):
        try:
            while not part.exists() or part.stat().st_size < expected:
                offset = part.stat().st_size if part.exists() else 0
                print(
                    f"{asset['filename']}: attempt {attempt}/{ATTEMPTS}, "
                    f"resume {offset}/{expected} bytes",
                    flush=True,
                )
                transfer(part, asset)
            verify(part, asset)
            # Keep user files intact if another process created the final file.
            if final.exists():
                verify(final, asset)
                print(f"Verified concurrently created {final}; partial file retained")
                return
            part.rename(final)
            print(f"Verified {final} ({expected} bytes), SHA-256 {asset['sha256']}")
            return
        except urllib.error.HTTPError as error:
            if error.code not in (408, 429, 500, 502, 503, 504):
                raise VerificationError(f"HTTP {error.code}; partial file retained") from error
            failure = error
        except (urllib.error.URLError, http.client.HTTPException, TimeoutError, OSError) as error:
            failure = error
        if attempt == ATTEMPTS:
            raise OSError(f"download failed after {ATTEMPTS} attempts: {failure}") from failure
        print(f"Transient download failure: {failure}; retrying", file=sys.stderr, flush=True)
        time.sleep(attempt)


def main() -> int:
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    assets = manifest["assets"]
    parser = argparse.ArgumentParser(
        description="Download and verify official RoboMaster CAD using pinned byte sizes and hashes.",
        epilog=(
            f"energy: {assets['energy']['size_bytes']:,} bytes; "
            f"field: {assets['field']['size_bytes']:,} bytes (~1.25 GB). "
            "The full field requires explicit --asset field or --asset all. "
            "Transfers use 30-second socket timeouts, at most 3 attempts, "
            "and resumable .part files. Existing invalid files are retained and rejected."
        ),
    )
    parser.add_argument("--asset", choices=("energy", "field", "all"), default="energy")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=ROOT / "build" / "fidelity-research" / "official",
        help="destination directory (default: repository build/fidelity-research/official)",
    )
    args = parser.parse_args()
    try:
        args.output_dir.mkdir(parents=True, exist_ok=True)
        for name in ("energy", "field") if args.asset == "all" else (args.asset,):
            fetch(args.output_dir, assets[name])
    except (VerificationError, OSError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("Interrupted; partial download retained for the next invocation", file=sys.stderr)
        return 130
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
