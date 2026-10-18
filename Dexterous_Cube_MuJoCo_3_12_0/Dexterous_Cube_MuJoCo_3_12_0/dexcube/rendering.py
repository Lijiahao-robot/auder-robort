import mujoco


class SceneRenderer:
    def __init__(self, task, width=960, height=540):
        self.width, self.height = width, height
        self.model = None
        self.renderer = None
        self.camera = mujoco.MjvCamera()
        self.option = mujoco.MjvOption()
        self._ensure(task.model)

    def _ensure(self, model):
        if self.model is model:
            return
        if self.renderer is not None:
            self.renderer.close()
        self.model = model
        self.renderer = mujoco.Renderer(model, width=self.width, height=self.height)

    def frame(self, model, data, mode='overview'):
        self._ensure(model)
        camera = self.camera
        camera.lookat[:] = [0, 0, .16]
        if mode == 'front':
            camera.distance, camera.azimuth, camera.elevation = .47, 90, -12
        elif mode == 'top':
            camera.distance, camera.azimuth, camera.elevation = .49, 90, -75
        else:
            camera.distance, camera.azimuth, camera.elevation = .51, 130, -28
        self.renderer.update_scene(data, camera=camera, scene_option=self.option)
        self.renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 0
        return self.renderer.render()

    def close(self):
        if self.renderer is not None:
            self.renderer.close()


class Demo:
    def __init__(self):
        self.stage = 0
        self.finished = False
        self.finish_at = None

    def update(self, task):
        if self.stage == 0 and task.grasped and task.data.time >= .8:
            task.enqueue(['R', 'U', "F'"])
            self.stage = 1
        elif self.stage == 1 and not task.current and not task.moves and task.phase == 'READY':
            self.stage = 2
            self.finish_at = float(task.data.time) + .7
        elif self.stage == 2 and task.data.time >= self.finish_at:
            task.restore()
            self.stage = 3
        elif self.stage == 3 and not task.current and not task.moves and task.phase == 'READY':
            self.stage = 4
            self.finish_at = float(task.data.time) + 1.0
        elif self.stage == 4 and task.data.time >= self.finish_at:
            self.finished = True
