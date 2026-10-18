from pathlib import Path
import numpy as np
from PIL import Image

root = Path(__file__).resolve().parents[1]
w, h = 1024, 512
phi = np.linspace(-np.pi, np.pi, w, endpoint=False)
theta = np.linspace(0, np.pi, h)
x = np.sin(theta)[:, None] * np.cos(phi)[None, :]
y = np.sin(theta)[:, None] * np.sin(phi)[None, :]
z = np.broadcast_to(np.cos(theta)[:, None], (h, w))
points = np.stack([x, y, z], axis=-1)
g = (1 + np.sqrt(5)) / 2
vertices = np.array([(0, a, b * g) for a in (-1, 1) for b in (-1, 1)] +
                    [(a, b * g, 0) for a in (-1, 1) for b in (-1, 1)] +
                    [(b * g, 0, a) for a in (-1, 1) for b in (-1, 1)])
vertices /= np.linalg.norm(vertices, axis=1)[:, None]
dots = np.sort(points @ vertices.T, axis=-1)
pixels = np.full((h, w, 3), [241, 244, 237], dtype=np.uint8)
pixels[dots[:, :, -1] > .93] = [23, 31, 31]
pixels[(dots[:, :, -1] - dots[:, :, -2]) < .009] = [170, 181, 170]
Image.fromarray(pixels).save(root / 'assets/football.png')
print('football.png generated')
