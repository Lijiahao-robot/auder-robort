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

ROOT=Path(__file__).resolve().parents[1]


def main():
    child=subprocess.Popen([sys.executable,str(ROOT/'start_web.py'),'--no-browser','--port','0','--width','640','--height','360','--fps','15'],
        cwd=ROOT,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,encoding='utf-8',env={**os.environ,'PYTHONIOENCODING':'utf-8'})
    messages,lines=queue.Queue(),[]
    def read():
        for line in child.stdout:lines.append(line);messages.put(line)
    threading.Thread(target=read,daemon=True).start()
    report,seq={'success':False,'checks':{}},0
    try:
        base=None;deadline=time.monotonic()+25
        while time.monotonic()<deadline:
            try:
                line=messages.get(timeout=.2)
                if 'http://127.0.0.1:' in line:base=line[line.index('http://127.0.0.1:'):].strip();break
            except queue.Empty:
                if child.poll() is not None:raise RuntimeError('Server exited: '+''.join(lines))
        if base is None:raise RuntimeError('Server startup timeout')
        def get(path):
            with urllib.request.urlopen(base+path,timeout=5) as response:return response.read()
        def state():return json.loads(get('/api/state'))
        def post(action,**extra):
            nonlocal seq
            seq+=1
            req=urllib.request.Request(base+'/api/control',json.dumps(dict(action=action,client='web-test',seq=seq,**extra)).encode(),{'Content-Type':'application/json'})
            with urllib.request.urlopen(req,timeout=5) as response:return json.loads(response.read())
        def wait(predicate,timeout=8):
            deadline=time.monotonic()+timeout
            while time.monotonic()<deadline:
                value=state()
                if value['error']:raise RuntimeError(value['error'])
                if predicate(value):return value
                time.sleep(.06)
            raise AssertionError('Timed out, last state: '+str(value))
        ready=lambda v:v['grasped'] and v['phase']=='READY' and not v['queue'] and v['current'] is None
        initial=wait(ready)
        assert initial['contacts']>=3
        assert b'cube-net' in get('/') and b'data-face="U"' in get('/') and b'parseSequence' in get('/controls.js')
        assert Image.open(io.BytesIO(get('/frame.jpg'))).size==(640,360)
        conn=http.client.HTTPConnection('127.0.0.1',int(base.rsplit(':',1)[1]),timeout=5)
        conn.request('GET','/stream.mjpg');response=conn.getresponse()
        assert response.status==200 and b'--frame' in response.read(150);conn.close()
        report['checks']['page_net_actual_jpeg_mjpeg']=True
        post('turn',move='R')
        moved=wait(lambda v:ready(v) and v['completed']==1)
        assert not moved['solved'] and moved['revision']>initial['revision']
        assert Image.open(io.BytesIO(get('/frame.jpg'))).size==(640,360)
        report['checks']['physical_turn_and_renderer_model_rebuild']=True
        post('undo')
        undone=wait(lambda v:ready(v) and v['completed']==2)
        assert undone['solved']
        report['checks']['undo_returns_solved']=True
        post('turn',move='U');wait(lambda v:v['current']=='U')
        post('pause');paused=wait(lambda v:v['paused'])
        time.sleep(.25);assert state()['time']==paused['time']
        post('pause');wait(lambda v:ready(v) and v['completed']==3)
        report['checks']['pause_mid_turn_and_resume']=True
        post('restore');restored=wait(lambda v:ready(v) and v['solved'],timeout=12)
        assert not restored['history']
        report['checks']['history_restore']=True
        post('pause');wait(lambda v:v['paused'])
        post('sequence',moves="F D'");queued=wait(lambda v:len(v['queue'])==2)
        assert queued['queue']==['F',"D'"]
        post('clear');wait(lambda v:not v['queue']);post('pause');wait(ready)
        report['checks']['sequence_queue_and_clear']=True
        stale=urllib.request.Request(base+'/api/control',json.dumps({'action':'turn','move':'U','client':'web-test','seq':1}).encode(),{'Content-Type':'application/json'})
        with urllib.request.urlopen(stale,timeout=5) as response:assert not json.loads(response.read())['accepted']
        try:post('turn',move='X');raise AssertionError('Invalid move accepted')
        except urllib.error.HTTPError as exc:assert exc.code==400
        bad=urllib.request.Request(base+'/api/control',b'{"action":"reset"}',{'Origin':'https://example.com','Content-Type':'application/json'})
        try:urllib.request.urlopen(bad,timeout=5);raise AssertionError('Foreign origin accepted')
        except urllib.error.HTTPError as exc:assert exc.code==403
        report['checks']['stale_invalid_and_foreign_requests_rejected']=True
        post('camera',mode='front');wait(lambda v:v['camera']=='front')
        report['checks']['camera_mode']=True
        post('release');released=wait(lambda v:not v['grasped'] and v['cube_position'][2]<.12)
        post('reset');reset=wait(ready)
        assert reset['solved'] and reset['completed']==0
        report['checks']['release_gravity_reset_regrasp']=True
        report.update({'success':True,'final_state':reset,'released_height_m':released['cube_position'][2],
                       'browser_visual_test':'not_run','windows_test':'not_run'})
    except Exception as exc:
        report['error']=repr(exc)
        raise
    finally:
        if child.poll() is None:
            if os.name=='nt':child.terminate()
            else:child.send_signal(signal.SIGINT)
        try:child.wait(timeout=8)
        except subprocess.TimeoutExpired:child.kill();child.wait()
        (ROOT/'results/validation_web.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        (ROOT/'results/validation_web.log').write_text(''.join(lines),encoding='utf-8')
        print(json.dumps(report,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
