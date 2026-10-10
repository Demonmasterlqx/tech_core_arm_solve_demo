#!/usr/bin/env python3
"""Verify the pinned selective snapshot; optionally restore absent source files."""
import argparse
import hashlib
import json
from pathlib import Path
import urllib.request


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--download", action="store_true", help="restore missing files only")
    args = parser.parse_args()
    root = Path(__file__).resolve().parent
    manifest = json.loads((root / "source_manifest.json").read_text())
    total = 0
    for item in manifest["files"]:
        target = root / item["path"]
        if not target.exists() and args.download:
            target.parent.mkdir(parents=True, exist_ok=True)
            temporary = target.with_name(target.name + ".part")
            try:
                with urllib.request.urlopen(item["url"], timeout=60) as response:
                    data = response.read()
                verify(item, data)
                temporary.write_bytes(data)
                temporary.replace(target)
            finally:
                temporary.unlink(missing_ok=True)
        data = target.read_bytes()
        verify(item, data)
        total += len(data)
    print(json.dumps({"commit": manifest["commit"], "verified_files": len(manifest["files"]),
                      "verified_bytes": total, "sha256_and_git_blob": "pass"}))


def verify(item, data):
    git_blob = hashlib.sha1(b"blob " + str(len(data)).encode() + b"\0" + data).hexdigest()
    if (len(data) != item["size"] or hashlib.sha256(data).hexdigest() != item["sha256"]
            or git_blob != item["git_blob"]):
        raise ValueError("Source mismatch: " + item["path"])


if __name__ == "__main__":
    main()
