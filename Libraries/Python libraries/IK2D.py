from math import sin, cos, pow, acos, atan, sqrt, pi

class IK2D:
    def __init__(self, L1, L2, yOffset = 0, xOffset = 0):
        self.L1 = L1
        self.L2 = L2

        self.yOffset = yOffset
        self.xOffset = xOffset

    def coordToAngles(self, x, y):
        x += self.xOffset
        y += self.yOffset

        q2 = pi - acos((pow(self.L1,2) + pow(self.L2,2) - pow(x,2) - pow(y,2))/2*self.L1*self.L2)
        q1 = atan(x/-y) - atan((self.L2*sin(q2))/(self.L1 + self.L2*cos(q2)))

        q2 *= 180/pi
        q1 *= 180/pi

        return q1,q2

