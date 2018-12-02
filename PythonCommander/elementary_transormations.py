from sympy import *


def tz(l):
    return Matrix([[1, 0, 0, 0],
                   [0, 1, 0, 0],
                   [0, 0, 1, l],
                   [0, 0, 0, 1]])


def tx(l):
    return Matrix([[1, 0, 0, l],
                   [0, 1, 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])


def rz(q):
    return Matrix([[cos(q), -sin(q), 0, 0],
                   [sin(q), cos(q), 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]
                   ])


def ry(q):
    return Matrix([[cos(q), 0, sin(q), 0],
                   [0, 1, 0, 0],
                   [-sin(q), 0, cos(q), 0],
                   [0, 0, 0, 1]
                   ])


def rx(q):
    return Matrix([[1, 0, 0, 0],
                   [0, cos(q), -sin(q), 0],
                   [0, sin(q), cos(q), 0],
                   [0, 0, 0, 1]
                   ])