import sympy as sp
import numpy as np

d2r = 2 * np.pi / 360.0
deg2rad = d2r

def Link_N(alpha, a, theta, d):
    alpha = alpha * deg2rad
    theta = theta * deg2rad

    ct = np.cos(theta)
    st = np.sin(theta)

    ca = np.cos(alpha)
    sa = np.sin(alpha)

    m = np.matrix([[ct      ,   -st     ,   0   ,   a],
                   [st * ca ,   ct * ca ,   -sa ,   -sa * d],
                   [st * sa ,   ct * sa ,   ca  ,   ca * d],
                   [0       ,   0       ,   0   ,   1]
                   ])
    return m

def Link_S(alpha, a, theta, d):
    ct = sp.cos(theta)
    st = sp.sin(theta)

    ca = sp.cos(alpha)
    sa = sp.sin(alpha)

    m = sp.Matrix([[ct      ,   -st     ,   0   ,   a],
                   [st * ca ,   ct * ca ,   -sa ,   -sa * d],
                   [st * sa ,   ct * sa ,   ca  ,   ca * d],
                   [0       ,   0       ,   0   ,   1]
                   ])
    return m