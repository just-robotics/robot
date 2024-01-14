import numpy as np


r = 0.07
lx = 0.37
ly = 0.3
vx = -0.25
vy = -0.1
wz = 0.2

w0 = 200
w1 = 200
w2 = 200
w3 = 200

kv = lx + ly
kw = 1 / (lx + ly)

Mv = np.array([[1, -1, -kv],
               [1, 1, kv],
               [1, 1, -kv],
               [1, -1, kv]])

Mw = np.array([[1, 1, 1, 1],
               [-1, 1, 1, -1],
               [-kw, kw, -kw, kw]])

V = np.array([[vx],
              [vy],
              [wz]])

W = np.array([[w0],
              [w1],
              [w2],
              [w3]])


def calcInverseKinematics(W):
    return r / 4 * np.dot(Mw, W)


def calcForwardKinematics(V):
    return 1 / r * np.dot(Mv, V)


def calcOdom():
    print(calcInverseKinematics(W))


def main():
    calcOdom()


if __name__ == '__main__':
    main()
