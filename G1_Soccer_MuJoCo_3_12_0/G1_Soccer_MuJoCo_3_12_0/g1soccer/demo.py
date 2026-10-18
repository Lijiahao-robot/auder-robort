class Demo:
    """Deterministic scripted walk/stop/kick sequence; no ball-state writes."""
    duration = 12.5

    def __init__(self, task):
        self.started = float(task.data.time)
        self.kicked = False

    def update(self, task, now):
        t = float(task.data.time) - self.started
        task.command(['forward'] if 1.2 <= t < 3.6 else [], now=now)
        if t >= 4.4 and not self.kicked:
            task.kick()
            self.kicked = True
        return t < self.duration
