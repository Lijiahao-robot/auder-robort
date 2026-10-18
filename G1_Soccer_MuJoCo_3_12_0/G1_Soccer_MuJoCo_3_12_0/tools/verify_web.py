"""Exercise the actual loopback server, renderer and command/score endpoints."""
from pathlib import Path
import http.client
import io
import json
import os
import queue
import signal
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request

from PIL import Image

ROOT = Path(__file__).resolve().parents[1]


def main():
    process = subprocess.Popen([sys.executable, str(ROOT / 'start_web.py'), '--no-browser', '--port', '0',
                                '--width', '640', '--height', '360', '--fps', '15'], cwd=ROOT,
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, encoding='utf-8',
                               env={**os.environ, 'PYTHONIOENCODING': 'utf-8'})
    messages, lines = queue.Queue(), []
    def reader():
        for line in process.stdout:
            lines.append(line)
            messages.put(line)
    threading.Thread(target=reader, daemon=True).start()
    report, seq = {'success': False, 'checks': {}}, 0
    try:
        base, deadline = None, time.monotonic() + 25
        while time.monotonic() < deadline:
            try:
                line = messages.get(timeout=.3)
                if 'http://127.0.0.1:' in line:
                    base = line[line.index('http://127.0.0.1:'):].strip()
                    break
            except queue.Empty:
                if process.poll() is not None:
                    raise RuntimeError('Server exited: ' + ''.join(lines))
        if base is None:
            raise RuntimeError('Server did not start within 25 seconds')

        def get(path):
            with urllib.request.urlopen(base + path, timeout=4) as response:
                return response.read()
        def state():
            return json.loads(get('/api/state'))
        def post(action, **extra):
            nonlocal seq
            seq += 1
            payload = dict(action=action, client='integration-test', seq=seq, **extra)
            request = urllib.request.Request(base + '/api/control', json.dumps(payload).encode(), {'Content-Type': 'application/json'})
            with urllib.request.urlopen(request, timeout=4) as response:
                return json.loads(response.read())

        assert b'id="stream"' in get('/') and b'data-code="KeyW"' in get('/')
        report['checks']['page_and_keyboard_served'] = True
        image = Image.open(io.BytesIO(get('/frame.jpg')))
        assert image.size == (640, 360)
        report['checks']['actual_mujoco_jpeg'] = True
        connection = http.client.HTTPConnection('127.0.0.1', int(base.rsplit(':', 1)[1]), timeout=4)
        connection.request('GET', '/stream.mjpg')
        response = connection.getresponse()
        assert response.status == 200 and b'--frame' in response.read(200)
        connection.close()
        report['checks']['mjpeg_stream'] = True

        initial = state()
        assert post('keys', keys=['forward'])['accepted']
        time.sleep(.5)
        moving = state()
        assert moving['robot'][0] > initial['robot'][0] + .015
        time.sleep(1.0)
        stopped = state()
        assert stopped['speed'] < .01 and stopped['command_timeout']
        report['checks']['walk_and_heartbeat_timeout'] = True
        assert post('pause')['accepted']
        time.sleep(.2)
        paused = state()
        time.sleep(.2)
        assert paused['paused'] and state()['time'] == paused['time']
        post('pause')
        report['checks']['pause_freezes_physics'] = True

        post('reset'); time.sleep(.25)
        post('keys', keys=['left', 'turn_left'])
        time.sleep(.5)
        turned = state()
        assert turned['robot'][1] > .01 and turned['robot'][2] > .01
        post('stop');time.sleep(.15)
        assert state()['speed'] < .01
        report['checks']['combined_strafe_turn_and_stop'] = True

        stale = {'action': 'keys', 'client': 'integration-test', 'seq': 1, 'keys': ['forward']}
        req = urllib.request.Request(base + '/api/control', json.dumps(stale).encode(), {'Content-Type': 'application/json'})
        with urllib.request.urlopen(req, timeout=4) as response:
            assert not json.loads(response.read())['accepted']
        report['checks']['stale_sequence_rejected'] = True
        try:
            post('keys', keys=['fly'])
            raise AssertionError('Invalid key was accepted')
        except urllib.error.HTTPError as exc:
            assert exc.code == 400
        req = urllib.request.Request(base + '/api/control', b'{"action":"stop"}', {'Origin': 'https://example.com', 'Content-Type': 'application/json'})
        try:
            urllib.request.urlopen(req, timeout=4)
            raise AssertionError('Foreign origin accepted')
        except urllib.error.HTTPError as exc:
            assert exc.code == 403
        report['checks']['invalid_input_and_foreign_origin_rejected'] = True

        post('reset');time.sleep(.35)
        post('ball');time.sleep(.1)
        post('kick')
        deadline = time.monotonic() + 10
        result = state()
        while result['goals'] < 1 and time.monotonic() < deadline:
            time.sleep(.15)
            result = state()
        assert result['goals'] == 1 and result['contact_count'] > 0
        report['checks']['web_kick_physical_contact_and_goal'] = True
        report.update({'success': True, 'goal_state': result, 'browser_visual_test': 'not_run', 'windows_test': 'not_run'})
    except Exception as exc:
        report['error'] = repr(exc)
        raise
    finally:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        (ROOT / 'results' / 'validation_web.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
        (ROOT / 'results' / 'validation_web.log').write_text(''.join(lines), encoding='utf-8')
        print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
