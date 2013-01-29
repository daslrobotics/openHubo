from collections import namedtuple
from numpy import *
from openravepy import RaveCreateModule,quatFromRotationMatrix
from rodrigues import rodrigues

SpaceNav=namedtuple('SpaceNav','x,y,z,rx,ry,rz,b0,b1')

class SpaceNav:

    def __init__(self,env,deadzone=array([20,20,20,20,20,20]),gains=array([.0001,.0001,.0001,pi/180,pi/180,pi/180])):
        self.env=env;
        self.module=RaveCreateModule(env,'SpaceNav')
        env.AddModule(self.module,'')
        self.deadzone=array(deadzone)
        self.gains=gains

    def get_raw_state(self):
        stData=self.module.SendCommand('GetState')
        data=[int(x) for x in stData.split(' ')]
        #Crude way: return raw int data
        return data

    def get_state(self):
        data=self.get_raw_state()
        for x in range(6):
            if abs(data[x])<self.deadzone[x]:
                data[x]=0
            else:
                #simply clip off deadzone from input, reducing max range
                data[x]=data[x]-(sign(data[x])*self.deadzone[x])

        return data

    def get_transform(self):
        data=array(self.get_state()[0:6])*self.gains
        deadzone=self.deadzone 

        T=eye(4)
        T[0,3]=data[2]
        T[1,3]=-data[0]
        T[2,3]=data[1]
        T[0:3,0:3]=rodrigues(array([1,-1,1])*data[[5,3,4]])
        return T

    def get_translation(self):
        data=array(self.get_state()[0:6])*self.gains
        deadzone=self.deadzone 
        T=array(zeros(3))
        T[0]=data[2]
        T[1]=-data[0]
        T[2]=data[1]
        return mat(T).T

    def get_affine(self):
        data=array(self.get_state()[0:6])*self.gains
        deadzone=self.deadzone 
        R=rodrigues(array([1,-1,1])*data[[5,3,4]])
        a=quatFromRotationMatrix(array(R))
        return mat(a).T

    def close(self):
        return self.module.SendCommand('CloseSpaceNav')

    def __del__(self):
        print "Sending CloseSpaceNav command..."
        self.close()
