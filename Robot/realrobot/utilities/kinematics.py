import numpy as np
from math import *
from adafruit_servokit import ServoKit


class Kinematic:

    def __init__(self):
        self.l1=56
        self.l2=20
        self.l3=100
        self.l4=100

        self.L = 149
        self.W = 75
        
        self.servoCeroAngle = [[83, 89, 165],[89, 82, 5],
                               [85, 91, 175],[86, 94, 7]]

        self.servoChannel = [[6,7,8],[9,10,11],
                             [0,1,2],[3,4,5]]
        
        self.pca = ServoKit(channels=16)
        self.Op=np.array([[self.L/2,-200,100,1],[self.L/2,-200,-100,1],[-self.L/2,-200,100,1],[-self.L/2,-200,-100,1]])

    def bodyIK(self,omega,phi,psi,xm,ym,zm):
        Rx = np.array([[1,0,0,0],
                    [0,np.cos(omega),-np.sin(omega),0],
                    [0,np.sin(omega),np.cos(omega),0],[0,0,0,1]])
        Ry = np.array([[np.cos(phi),0,np.sin(phi),0],
                    [0,1,0,0],
                    [-np.sin(phi),0,np.cos(phi),0],[0,0,0,1]])
        Rz = np.array([[np.cos(psi),-np.sin(psi),0,0],
                    [np.sin(psi),np.cos(psi),0,0],[0,0,1,0],[0,0,0,1]])
        Rxyz = Rx.dot(Ry.dot(Rz))

        T = np.array([[0,0,0,xm],[0,0,0,ym],[0,0,0,zm],[0,0,0,0]])
        Tm = T+Rxyz

        sHp=np.sin(pi/2)
        cHp=np.cos(pi/2)
        (L,W)=(self.L,self.W)

        return([Tm.dot(np.array([[cHp,0,sHp,L/2],[0,1,0,0],[-sHp,0,cHp,W/2],[0,0,0,1]])),
                Tm.dot(np.array([[cHp,0,sHp,L/2],[0,1,0,0],[-sHp,0,cHp,-W/2],[0,0,0,1]])),
                Tm.dot(np.array([[cHp,0,sHp,-L/2],[0,1,0,0],[-sHp,0,cHp,W/2],[0,0,0,1]])),
                Tm.dot(np.array([[cHp,0,sHp,-L/2],[0,1,0,0],[-sHp,0,cHp,-W/2],[0,0,0,1]]))])

    def legIK(self,point):
        (x,y,z)=(point[0],point[1],point[2])
        (l1,l2,l3,l4)=(self.l1,self.l2,self.l3,self.l4)
        try:        
            F=sqrt(x**2+y**2-l1**2)
        except ValueError:
            print("Error in legIK with x {} y {} and l1 {}".format(x,y,l1))
            F=l1
        G=F-l2  
        H=sqrt(G**2+z**2)
        theta1=-atan2(y,x)-atan2(F,-l1)
        
        D=(H**2-l3**2-l4**2)/(2*l3*l4)
        try:        
            theta3=acos(D) 
        except ValueError:
            print("Error in legIK with x {} y {} and D {}".format(x,y,D))
            theta3=0
        theta2=atan2(z,G)-atan2(l4*sin(theta3),l3+l4*cos(theta3))
        
        return[theta1,theta2,theta3]

    def setRightLeg(self,theta, leg):
        if theta[0]>=pi/4:
            print("Error shoulder servo in dangerous position")
            return 0
        for i in range(3):
            self.pca.servo[self.servoChannel[leg][i]].angle = self.servoCeroAngle[leg][i]+theta[i]*180/pi
        
    def setLeftLeg(self,theta, leg):
        if theta[0]<=-pi/4:
            print("Error shoulder servo in dangerous position")
            return 0
        for i in range(3):
            self.pca.servo[self.servoChannel[leg][i]].angle = self.servoCeroAngle[leg][i]-theta[i]*180/pi
            

    def setLegPair(self,Tl,Tr,Ll,Lr, legPair):
        Ix=np.array([[-1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
        self.setLeftLeg(self.legIK(np.linalg.inv(Tl).dot(Ll)), legPair[0])
        self.setRightLeg(self.legIK(Ix.dot(np.linalg.inv(Tr).dot(Lr))), legPair[1])
        
    def setRobot(self,Lp,angles,center):
        (omega,phi,psi)=angles
        (xm,ym,zm)=center
        
        FP=[0,0,0,1]
        (Tlf,Trf,Tlb,Trb)= self.bodyIK(omega,phi,psi,xm,ym,zm)
        CP=[x.dot(FP) for x in [Tlf,Trf,Tlb,Trb]]
        """
        CPs=[CP[x] for x in [0,1,3,2,0]]
        plt.plot([x[0] for x in CPs],[x[2] for x in CPs],[x[1] for x in CPs], 'bo-', lw=2)
        """
        Lp = self.Op+Lp
        self.setLegPair(Tlf,Trf,Lp[0],Lp[1], legPair = [0,1])
        self.setLegPair(Tlb,Trb,Lp[2],Lp[3], legPair = [2,3])

    def calcIK(self,Lp,angles,center):
        (omega,phi,psi)=angles
        (xm,ym,zm)=center
        
        (Tlf,Trf,Tlb,Trb)= self.bodyIK(omega,phi,psi,xm,ym,zm)

        Ix=np.array([[-1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
        return np.array([self.legIK(np.linalg.inv(Tlf).dot(Lp[0])),
        self.legIK(Ix.dot(np.linalg.inv(Trf).dot(Lp[1]))),
        self.legIK(np.linalg.inv(Tlb).dot(Lp[2])),
        self.legIK(Ix.dot(np.linalg.inv(Trb).dot(Lp[3])))])

def trayectory(t):
    a = 1/50*pi
    return -sin(a*t), -sin(a*t), -cos(a*t)/(1+cos(a*t)*cos(a*t)) + 0.5
"""
def update(x,y,z):
    points.set_data
"""


if __name__=="__main__":    
    x, y, z = 0,0,0
    Lp=np.array([[x,y,z,0],[x,y,z,0],[x,y,z,0],[x,y,z,0]])
    Kinematic().setRobot(Lp,(0,0,0),(0,0,0))
