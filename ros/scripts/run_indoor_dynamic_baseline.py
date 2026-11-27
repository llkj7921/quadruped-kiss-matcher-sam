#!/usr/bin/env python3
"""Run the recorded indoor point -> adaptive SOR -> cluster baseline offline."""
import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path

from capture_visibility_evidence import source_hashes


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def commands(session, output, visibility, scripts):
    point = output / '01_point'
    outlier = output / '02_outlier'
    group = output / '03_cluster'
    return [
        [str(visibility), str(session), str(point), '--output-voxel', '0.05'],
        [sys.executable, str(scripts / 'adaptive_outlier_review.py'), '--map', str(point / 'clean_map.pcd'), '--output', str(outlier), '--neighbors', '20', '--local-neighbors', '100', '--sigma-factor', '2.5', '--ratio', '1.5', '--threshold-floor', '0.04', '--component-radius', '0.12', '--component-max-points', '5', '--display-max-z', '1.5'],
        [sys.executable, str(scripts / 'cluster_visibility_review.py'), '--map', str(outlier / 'kept.pcd'), '--keyframes', str(session / 'keyframes'), '--trajectory', str(session / 'optimized_trajectory_tum.txt'), '--timestamps', str(session / 'timestamps.txt'), '--output', str(group), '--radius', '0.18', '--min-points', '10', '--max-points', '2000', '--sample-count', '40', '--angular-resolution', '0.01', '--range-margin', '0.4', '--max-range', '30', '--min-valid-rays', '3', '--minimum-coverage', '0.2', '--free-fraction', '0.65', '--occupied-fraction', '0.5', '--min-occupied-frames', '2', '--min-free-frames', '3', '--separation', '1', '--temporal-mode', 'bidirectional', '--min-viewpoint-separation', '0.5', '--min-evidence-time-gap', '2', '--display-max-z', '1.5'],
    ]


def verify_point_parameters(path):
    values = dict(line.split(':', 1) for line in path.read_text().splitlines() if ':' in line)
    expected = {'angular_resolution_rad': .01, 'range_margin_m': .4, 'maximum_range_m': 30., 'minimum_time_separation_s': .5, 'minimum_free_observations': 3., 'free_to_occupied_ratio': 2., 'output_voxel_resolution_m': .05}
    for key, value in expected.items():
        if key not in values or abs(float(values[key]) - value) > 1e-9:
            raise ValueError(f'visibility executable does not use recorded baseline parameter {key}={value}')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--session', type=Path, required=True, help='Saved map product with keyframes, optimized trajectory and timestamps')
    p.add_argument('--output', type=Path, required=True, help='New output directory')
    p.add_argument('--visibility-executable', type=Path, required=True, help='Built visibility_map_filter executable')
    a = p.parse_args()
    session, output, executable = (x.resolve() for x in (a.session, a.output, a.visibility_executable))
    if not session.is_dir() or not executable.is_file():
        p.error('session or visibility executable does not exist')
    if output.exists() or output == session or session in output.parents or output in session.parents:
        p.error('output must be new and separate from the input session')
    scripts = Path(__file__).resolve().parent
    before = source_hashes(session)
    runs = commands(session, output, executable, scripts)
    tool_hashes = {str(x): digest(x) for x in [executable, scripts / 'adaptive_outlier_review.py', scripts / 'cluster_visibility_review.py', Path(__file__).resolve()]}
    output.mkdir(parents=True)
    manifest = {'status': 'running', 'session': str(session), 'source_hashes': before, 'tool_sha256': tool_hashes, 'commands': runs, 'warning': 'Offline experimental candidate filtering; inspect removed and retained points before any downstream use.'}
    path = output / 'manifest.json'
    path.write_text(json.dumps(manifest, indent=2) + '\n')
    try:
        for index, run in enumerate(runs):
            print('Running:', ' '.join(run), flush=True)
            subprocess.run(run, check=True)
            if index == 0:
                verify_point_parameters(output / '01_point' / 'report.yaml')
        if source_hashes(session) != before:
            raise RuntimeError('input session changed during processing')
        manifest['final_pcd'] = str(output / '03_cluster' / 'proposed_kept.pcd')
        manifest['final_sha256'] = digest(Path(manifest['final_pcd']))
        manifest['status'] = 'complete'
    except BaseException as error:
        manifest['status'] = 'failed'
        manifest['error'] = str(error)
        raise
    finally:
        path.write_text(json.dumps(manifest, indent=2) + '\n')
    print('Final map:', manifest['final_pcd'], flush=True)


if __name__ == '__main__':
    main()
