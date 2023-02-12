from IK2D import IK2D
from math import sqrt

IK = IK2D(L1=1,L2=1, yOffset=-sqrt(2))

print(IK.coordToAngles(x=0, y=0))