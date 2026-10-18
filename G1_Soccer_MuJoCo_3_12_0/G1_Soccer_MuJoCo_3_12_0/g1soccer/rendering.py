import mujoco


class SceneRenderer:
    def __init__(self, task, width=960, height=540):
        self.task = task
        self.renderer = mujoco.Renderer(task.model, width=width, height=height)
        self.camera = mujoco.MjvCamera()
        self.option = mujoco.MjvOption()
        self.option.geomgroup[3] = 0

    def frame(self, mode='field', data=None, robot_pose=None):
        t, cam = self.task, self.camera
        xy, heading = (t.base_xy, t.heading) if robot_pose is None else (robot_pose[:2], robot_pose[2])
        if mode == 'follow':
            cam.lookat[:] = [xy[0] + .4, xy[1], .55]
            cam.distance, cam.azimuth, cam.elevation = 4.3, 135, -22
        elif mode == 'behind':
            cam.lookat[:] = [xy[0] + .55, xy[1], .5]
            cam.distance, cam.azimuth, cam.elevation = 4.0, 180 + heading * 57.2957795, -18
        else:
            cam.lookat[:] = [.35, 0., .35]
            cam.distance, cam.azimuth, cam.elevation = 8.4, 140, -29
        self.renderer.update_scene(t.data if data is None else data, camera=cam, scene_option=self.option)
        self.renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 0
        return self.renderer.render()

    def close(self):
        self.renderer.close()
