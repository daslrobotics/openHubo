from collections import namedtuple
from numpy import abs, sign,array
from openravepy import RaveCreateModule

SpaceNav=namedtuple('SpaceNav','x,y,z,rx,ry,rz,b0,b1')

class SpaceNav:

    def __init__(self,env,deadzone=array([20,20,20,20,20,20])):
        self.env=env;
        self.module=RaveCreateModule(env,'SpaceNav')
        env.AddModule(self.module,'')
        self.deadzone=array(deadzone)

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

