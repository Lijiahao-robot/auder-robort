"""Regression checks for model loading from Unicode/space-containing paths."""
import os
os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
os.environ.setdefault('OMP_NUM_THREADS', '1')
os.environ.setdefault('PYTHONUTF8', '1')

import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time

import mujoco
import numpy as np

from atlas_sim.model_loader import load_atlas_model

ROOT = Path(__file__).resolve().parent


def main():
    begun = time.perf_counter()
    checks = []
    model = load_atlas_model()
    assert (model.nq, model.nv, model.nu) == (34, 33, 27)
    assert model.nmesh == 22 and model.mesh_vert.size > 0
    assert np.isclose(float(np.sum(model.body_mass)), 171.6626)
    checks.append({'name': 'compiled_model_and_mesh_contract', 'passed': True})
    with tempfile.TemporaryDirectory(prefix='atlas_loader_') as temporary:
        temp = Path(temporary)
        copied = temp/'开发 机器人 仿真'/'Atlas_MuJoCo_3_12'
        copied.mkdir(parents=True)
        shutil.copy2(ROOT/'run_atlas.py', copied/'run_atlas.py')
        shutil.copytree(ROOT/'atlas_sim', copied/'atlas_sim', ignore=shutil.ignore_patterns('__pycache__'))
        shutil.copytree(ROOT/'models', copied/'models')
        unrelated = temp/'unrelated working directory'
        unrelated.mkdir()
        cwd = Path.cwd()
        try:
            os.chdir(unrelated)
            loaded = load_atlas_model(copied/'models')
        finally:
            os.chdir(cwd)
        assert loaded.nu == 27 and loaded.nv == 33
        checks.append({'name': 'unicode_spaces_and_unrelated_working_directory', 'passed': True})
        command = [sys.executable, str(copied/'run_atlas.py'), '--headless', '--mode', 'walk',
                   '--seconds', '15', '--report', str(copied/'results'/'loading_walk_report.json')]
        process = subprocess.run(command, cwd=unrelated, capture_output=True, text=True,
                                 encoding='utf8', timeout=60)
        assert process.returncode == 0, process.stdout+'\n'+process.stderr
        run = json.loads((copied/'results'/'loading_walk_report.json').read_text(encoding='utf8'))
        assert run['success'] and not run['state']['fallen'] and run['state']['step_count'] >= 3
        checks.append({'name': 'actual_entrypoint_walks_from_unicode_path', 'passed': True,
                       'simulation_seconds': run['state']['time'], 'steps': run['state']['step_count']})
        mesh = copied/'models'/'meshes'/'pelvis.stl'
        saved_mesh = mesh.with_suffix('.stl.saved')
        mesh.rename(saved_mesh)
        try:
            try:
                load_atlas_model(copied/'models')
            except FileNotFoundError as exc:
                assert 'pelvis.stl' in str(exc) and '网格' in str(exc)
            else:
                raise AssertionError('A missing mesh must be reported before compilation.')
        finally:
            saved_mesh.rename(mesh)
        checks.append({'name': 'missing_mesh_has_actionable_diagnostic', 'passed': True})
        (copied/'models'/'atlas.xml').unlink()
        process = subprocess.run([sys.executable, str(copied/'run_atlas.py'), '--headless', '--seconds', '1'],
                                 cwd=unrelated, capture_output=True, text=True, encoding='utf8', timeout=15)
        assert process.returncode == 2
        assert 'atlas.xml' in process.stderr and '未找到模型' in process.stderr
        assert 'Traceback' not in process.stderr
        checks.append({'name': 'missing_xml_is_reported_cleanly_by_entrypoint', 'passed': True})
    report = {'all_passed': True, 'case_count': len(checks), 'checks': checks,
              'mujoco_version': mujoco.__version__, 'platform': sys.platform,
              'windows_native_tested': False, 'wall_seconds': round(time.perf_counter()-begun, 3),
              'method': 'Python file reads, in-memory MjVfs; real headless physics from copied Unicode project path.'}
    target = ROOT/'results'/'model_loading_verification.json'
    target.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf8')
    for check in checks:
        print('PASS', check['name'])
    print('Report:', target)


if __name__ == '__main__':
    main()
