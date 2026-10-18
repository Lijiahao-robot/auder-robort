from __future__ import annotations
from dataclasses import dataclass
import random
import re

import numpy as np

FACE_NORMALS = {'U': (0, 0, 1), 'R': (1, 0, 0), 'F': (0, -1, 0),
                'D': (0, 0, -1), 'L': (-1, 0, 0), 'B': (0, 1, 0)}
FACE_COLORS = {'U': '#f4f1dd', 'R': '#e95643', 'F': '#39b97b',
               'D': '#efca47', 'L': '#ef9643', 'B': '#4488dc'}
PITCH = .022
HALF = .0106


def parse_move(token):
    if not isinstance(token, str) or not re.fullmatch(r"[URFDLB](?:'|2)?", token):
        raise ValueError("动作格式为 U/R/F/D/L/B，可加 ' 或 2")
    return token[0], (2 if token.endswith('2') else (-1 if token.endswith("'") else 1))


def inverse_move(token):
    face, turns = parse_move(token)
    return face + ('2' if turns == 2 else ("'" if turns == 1 else ''))


def rotation(normal, turns):
    # Clockwise as viewed from outside the selected face = negative RHS angle.
    n = np.asarray(normal, dtype=int)
    cross = np.array([[0, -n[2], n[1]], [n[2], 0, -n[0]], [-n[1], n[0], 0]])
    quarter = np.outer(n, n) - cross
    return np.linalg.matrix_power(quarter, turns % 4).astype(int)


@dataclass
class Cubie:
    name: str
    home: np.ndarray
    position: np.ndarray
    orientation: np.ndarray


class CubeState:
    def __init__(self):
        self.cubies = []
        for x in (-1, 0, 1):
            for y in (-1, 0, 1):
                for z in (-1, 0, 1):
                    if x == y == z == 0:
                        continue
                    p = np.array([x, y, z], dtype=int)
                    self.cubies.append(Cubie(f'piece_{x+1}{y+1}{z+1}', p.copy(), p.copy(), np.eye(3, dtype=int)))
        self.history = []

    def selected(self, face):
        normal = np.array(FACE_NORMALS[face])
        return [p for p in self.cubies if int(p.position @ normal) == 1]

    def apply(self, token):
        face, turns = parse_move(token)
        transform = rotation(FACE_NORMALS[face], turns)
        selected = self.selected(face)
        for piece in selected:
            piece.position = transform @ piece.position
            piece.orientation = transform @ piece.orientation
        self.history.append(token)

    def facelets(self):
        result = {face: [None] * 9 for face in FACE_NORMALS}
        # Face-local horizontal and vertical axes, viewed from outside.
        axes = {'U': ((1, 0, 0), (0, 1, 0)), 'D': ((1, 0, 0), (0, -1, 0)),
                'F': ((1, 0, 0), (0, 0, 1)), 'B': ((-1, 0, 0), (0, 0, 1)),
                'R': ((0, 1, 0), (0, 0, 1)), 'L': ((0, -1, 0), (0, 0, 1))}
        for piece in self.cubies:
            for home_face, normal in FACE_NORMALS.items():
                normal = np.array(normal)
                if int(piece.home @ normal) != 1:
                    continue
                outward = piece.orientation @ normal
                face = next(f for f, n in FACE_NORMALS.items() if np.array_equal(outward, n))
                horizontal, vertical = map(np.array, axes[face])
                column = int(piece.position @ horizontal) + 1
                row = 1 - int(piece.position @ vertical)
                result[face][row * 3 + column] = FACE_COLORS[home_face]
        if any(v is None for colors in result.values() for v in colors):
            raise RuntimeError('Invalid sticker permutation')
        return result

    @property
    def solved(self):
        return all(len(set(colors)) == 1 for colors in self.facelets().values())

    def signature(self):
        return tuple(tuple(self.facelets()[face]) for face in FACE_NORMALS)

    def inverse_history(self):
        return [inverse_move(move) for move in reversed(self.history)]


def scramble_moves(count=6, seed=None):
    rng = random.Random(seed)
    moves, prior = [], None
    for _ in range(count):
        face = rng.choice([f for f in FACE_NORMALS if f != prior])
        moves.append(face + rng.choice(['', "'", '2']))
        prior = face
    return moves
