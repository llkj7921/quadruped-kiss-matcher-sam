#!/usr/bin/env python3
"""Run the offline point-evidence exporter with immutable-input checksums."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess


def sha256_files(paths):
    digest = hashlib.sha256()
    for path in paths:
        digest.update(path.name.encode('utf-8') + b'\0')
        with path.open('rb') as stream:
            for block in iter(lambda: stream.read(1048576), b''):
                digest.update(block)
    return digest.hexdigest()


def source_hashes(session):
    frames = sorted((session / 'keyframes').glob('*.pcd'))
    if not frames:
        raise ValueError('no keyframes')
    return {'keyframes': sha256_files(frames), 'trajectory': sha256_files([session / 'optimized_trajectory_tum.txt']), 'timestamps': sha256_files([session / 'timestamps.txt'])}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--session', required=True, type=Path)
    p.add_argument('--exporter', required=True, type=Path)
    p.add_argument('--output', required=True, type=Path)
    a = p.parse_args()
    if a.output.exists():
        p.error('output already exists')
    before = source_hashes(a.session)
    subprocess.run([str(a.exporter.resolve()), str(a.session.resolve()), str(a.output.resolve())], check=True)
    after = source_hashes(a.session)
    if before != after:
        raise RuntimeError('input data changed during export; refuse to certify evidence')
    manifest = {'session': str(a.session.resolve()), 'source_hashes': before, 'evidence_sha256_name_and_bytes': sha256_files([a.output / 'point_evidence.tsv']), 'exporter_sha256_name_and_bytes': sha256_files([a.exporter])}
    (a.output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print(json.dumps(manifest, indent=2))


if __name__ == '__main__':
    main()
