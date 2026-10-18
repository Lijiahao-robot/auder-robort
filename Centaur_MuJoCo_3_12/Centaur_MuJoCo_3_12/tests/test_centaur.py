import ast
import http.client
import io
import json
import math
import os
from pathlib import Path
import re
import threading
import time
import unittest
from html.parser import HTMLParser

if os.name!='nt' and not os.environ.get('DISPLAY'):
    os.environ.setdefault('MUJOCO_GL','egl')

import mujoco
import numpy as np
from PIL import Image
from centaur.model import ROOT,LEGS,UPPER_LEG,LOWER_LEG
from centaur.simulation import Simulation,leg_ik
from centaur.web import LiveSimulation,WebServer,validate_command


class PhysicsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sim=Simulation()
    def setUp(self):
        self.sim.configure('stand',.12,True)
        self.sim.reset()
    def test_exact_mujoco_version_and_free_base_topology(self):
        self.assertEqual(mujoco.__version__,'3.12.0')
        self.assertEqual(self.sim.model.nu,25)
        self.assertEqual(self.sim.model.nq,32)
        self.assertEqual(sum(self.sim.model.jnt_type==mujoco.mjtJoint.mjJNT_FREE),1)
        self.assertEqual(len(self.sim.foot_ids),4)
    def test_leg_inverse_kinematics_reconstructs_reachable_targets(self):
        for target in ([0,0,-.54],[.08,.04,-.51],[-.10,-.04,-.52]):
            a,h,k=leg_ik(target)
            depth=UPPER_LEG*math.cos(h)+LOWER_LEG*math.cos(h+k)
            actual=[-UPPER_LEG*math.sin(h)-LOWER_LEG*math.sin(h+k),math.sin(a)*depth,-math.cos(a)*depth]
            np.testing.assert_allclose(actual,target,atol=1e-8)
    def test_stand_walk_wave_dance_scenarios(self):
        reports=[]
        for mode,speed in [('stand',.12),('walk',.06),('walk',.12),('walk',.20),('wave',.12),('dance',.12)]:
            with self.subTest(mode=mode,speed=speed):
                self.sim.configure(mode,speed,True);self.sim.reset();self.sim.step(6000)
                r=self.sim.snapshot();reports.append(r)
                self.assertFalse(r['fallen'])
                self.assertGreater(r['min_base_height_m'],.40)
                self.assertLess(r['max_tilt_deg'],30)
                self.assertLess(r['tracking_rms_rad'],.15)
                self.assertLess(r['max_joint_limit_violation_rad'],.05)
                self.assertLessEqual(r['peak_torque_limit_fraction'],1)
                self.assertGreater(r['contact_frame_fraction'],.90)
                self.assertFalse(np.any(self.sim.data.warning.number))
                if mode=='walk':self.assertGreater(r['displacement_m'][0],.25)
        if os.environ.get('CENTAUR_VALIDATION_DIR'):
            output=Path(os.environ['CENTAUR_VALIDATION_DIR']);output.mkdir(exist_ok=True,parents=True)
            (output/'physics_scenarios.json').write_text(json.dumps(reports,ensure_ascii=False,indent=2),encoding='utf-8')
    def test_disabling_assistance_applies_no_external_wrench(self):
        self.sim.configure('stand',.12,False);self.sim.reset();self.sim.step(1000)
        np.testing.assert_array_equal(self.sim.data.xfrc_applied,np.zeros_like(self.sim.data.xfrc_applied))
        self.assertEqual(self.sim.snapshot()['peak_external_assist_force_N'],0)
        self.assertFalse(self.sim.snapshot()['unassisted_locomotion_verified'])
    def test_manual_reverse_lateral_turn_and_combined_commands(self):
        reports=[]
        for velocity in [(-.12,0,0),(0,.08,0),(0,0,.30),(.085,.056,.30)]:
            with self.subTest(velocity=velocity):
                self.sim.configure('manual',.12,True);self.sim.reset();self.sim.velocity(*velocity);self.sim.step(4000)
                r=self.sim.snapshot();r['requested_velocity']=list(velocity);reports.append(r)
                self.assertFalse(r['fallen']);self.assertLess(r['max_tilt_deg'],30)
                self.assertLess(r['max_joint_limit_violation_rad'],.05)
                if velocity[0]<0:self.assertLess(r['displacement_m'][0],-.15)
                if velocity[0]==0 and velocity[1]>0:self.assertGreater(r['displacement_m'][1],.15)
                if velocity[0]==0 and velocity[1]==0:
                    q=self.sim.data.qpos[3:7]
                    yaw=math.atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]**2+q[3]**2))
                    self.assertGreater(yaw,1.0)
        if os.environ.get('CENTAUR_VALIDATION_DIR'):
            output=Path(os.environ['CENTAUR_VALIDATION_DIR']);output.mkdir(exist_ok=True,parents=True)
            (output/'manual_scenarios.json').write_text(json.dumps(reports,ensure_ascii=False,indent=2),encoding='utf-8')
    def test_all_joint_references_remain_in_range_when_switching_modes(self):
        for mode in ('walk','wave','dance','stand'):
            self.sim.configure(mode,.2,True)
            for _ in range(600):
                self.sim.step()
                self.assertTrue(np.all(self.sim.last_ref>=self.sim.limits[:,0]+.024))
                self.assertTrue(np.all(self.sim.last_ref<=self.sim.limits[:,1]-.024))
    def test_reset_restores_initial_free_base_and_statistics(self):
        self.sim.configure('walk',.12,True);self.sim.step(1000);self.sim.reset()
        np.testing.assert_allclose(self.sim.data.qpos[:3],[0,0,.62])
        self.assertEqual(self.sim.steps,0)
        self.assertFalse(self.sim.fallen)
    def test_invalid_speeds_commands_and_assistance_types_rejected(self):
        for values in [('missing',.12,True),('walk',.21,True),('walk',float('nan'),True),('walk',True,True),('stand',.12,'false')]:
            with self.assertRaises(ValueError):self.sim.configure(*values)
        for body in [{'action':'velocity','vx':.5,'vy':0,'yaw':0},{'action':'camera','view':'unknown'},{'action':'run_code'},{'action':'configure','mode':'walk','speed':.1,'assist':'yes'}]:
            with self.assertRaises(ValueError):validate_command(body)


class LiveTests(unittest.TestCase):
    def wait_for(self,live,predicate,timeout=8):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            state=live.snapshot()
            if predicate(state):return state
            time.sleep(.04)
        self.fail('Live simulation state was not reached')
    def test_real_render_thread_run_pause_and_reset(self):
        live=LiveSimulation()
        try:
            live.start()
            self.assertEqual(Image.open(io.BytesIO(live.frame())).size,(960,640))
            live.command({'action':'configure','mode':'walk','speed':.12,'assist':True})
            self.wait_for(live,lambda s:s['running'] and s['simulation_time_s']>.35)
            live.command({'action':'pause'})
            paused=self.wait_for(live,lambda s:not s['running'])
            time.sleep(.35)
            self.assertEqual(live.snapshot()['simulation_time_s'],paused['simulation_time_s'])
            live.command({'action':'reset'})
            reset=self.wait_for(live,lambda s:s['simulation_time_s']==0)
            self.assertEqual(reset['physics_steps'],0)
        finally:live.close()
        self.assertFalse(live.thread.is_alive())
    def test_keyboard_velocity_expires_without_refresh(self):
        live=LiveSimulation()
        try:
            live.start();live.command({'action':'velocity','vx':.12,'vy':0,'yaw':0})
            self.wait_for(live,lambda s:s.get('keyboard_velocity',[0])[0]>.1)
            state=self.wait_for(live,lambda s:s.get('mode')=='manual' and s.get('keyboard_velocity')==[0,0,0])
            self.assertEqual(state['keyboard_velocity'],[0,0,0])
        finally:live.close()


class FakeLive:
    def __init__(self):self.commands=[]
    def snapshot(self):return {'mujoco_version':'3.12.0','assist_enabled':True,'hardware_connected':False}
    def frame(self):return b'jpeg-test-fixture'
    def command(self,body):self.commands.append(validate_command(body));return {'accepted':True}


class WebTests(unittest.TestCase):
    def setUp(self):
        self.live=FakeLive();self.server=WebServer(('127.0.0.1',0),self.live)
        self.thread=threading.Thread(target=self.server.serve_forever,kwargs={'poll_interval':.02},daemon=True);self.thread.start()
    def tearDown(self):
        self.server.shutdown();self.server.server_close();self.thread.join(2)
    def request(self,method,path,body=None,headers=None):
        connection=http.client.HTTPConnection('127.0.0.1',self.server.server_port,timeout=4)
        headers=dict(headers or {});raw=None
        if body is not None:raw=json.dumps(body).encode();headers['Content-Type']='application/json'
        connection.request(method,path,raw,headers);response=connection.getresponse();raw=response.read()
        value=json.loads(raw) if response.getheader('Content-Type','').startswith('application/json') else raw
        result=response.status,value,dict(response.getheaders());connection.close();return result
    def test_assets_frame_and_state_with_security_headers(self):
        for path in ('/','/app.js','/input.js','/app.css','/api/frame','/api/state'):
            status,value,headers=self.request('GET',path);self.assertEqual(status,200);self.assertTrue(value)
            self.assertIn("frame-ancestors 'none'",headers['Content-Security-Policy'])
    def test_commands_require_session_key(self):
        body={'action':'configure','mode':'walk','speed':.12,'assist':True}
        self.assertEqual(self.request('POST','/api/control',body)[0],403)
        _,session,_=self.request('GET','/api/session')
        self.assertEqual(self.request('POST','/api/control',body,{'X-Simulation-Key':session['key']})[0],202)
        self.assertEqual(len(self.live.commands),1)
    def test_foreign_origin_host_and_path_traversal_rejected(self):
        for headers in ({'Origin':'https://unrelated.example'},{'Host':'unrelated.example'}):
            self.assertEqual(self.request('GET','/api/session',headers=headers)[0],403)
        self.assertEqual(self.request('GET','/../assets/centaur.xml')[0],404)
    def test_invalid_control_does_not_reach_worker(self):
        body={'action':'velocity','vx':5,'vy':0,'yaw':0}
        self.assertEqual(self.request('POST','/api/control',body,{'X-Simulation-Key':self.server.key})[0],400)
        self.assertFalse(self.live.commands)
    def test_report_download_preserves_assistance_label(self):
        status,value,headers=self.request('GET','/api/report')
        self.assertEqual(status,200);self.assertTrue(value['assist_enabled']);self.assertFalse(value['hardware_connected'])
        self.assertIn('attachment',headers['Content-Disposition'])


class PackagingContractTests(unittest.TestCase):
    def test_html_ids_unique_and_script_targets_exist(self):
        class Collector(HTMLParser):
            def __init__(self):super().__init__();self.ids=[]
            def handle_starttag(self,tag,attrs):self.ids.extend(v for k,v in attrs if k=='id')
        parser=Collector();parser.feed((ROOT/'web/index.html').read_text())
        self.assertEqual(len(parser.ids),len(set(parser.ids)))
        targets=set(re.findall(r"\$\(['\"]([^'\"]+)['\"]\)",(ROOT/'web/app.js').read_text()))
        self.assertTrue(targets.issubset(set(parser.ids)))
    def test_bootstrap_imports_only_standard_library_before_install(self):
        tree=ast.parse((ROOT/'bootstrap.py').read_text())
        roots=[]
        for node in tree.body:
            if isinstance(node,ast.Import):roots.extend(x.name.split('.')[0] for x in node.names)
            elif isinstance(node,ast.ImportFrom):roots.append(node.module.split('.')[0])
        self.assertFalse(set(roots)&{'mujoco','PIL','numpy','imageio','centaur'})
        self.assertIn('mujoco==3.12.0',(ROOT/'requirements.txt').read_text())
