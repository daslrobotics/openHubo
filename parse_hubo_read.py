import re
import sys
from numpy import pi,array
#import matplotlib.pyplot s plt
#import scipy.spatial as spatial
from openhubo import mapping

class hubo_state:

    def __init__(self,t=0.0):
        self.t=t
        self.joints={}
        self.sensors={}

    def insert_joint(self,name,data):
        #assumes the type is correct!
        self.joints.setdefault(name,data)

    def insert_sensor(self,name,sensor):
        #assumes the type is correct!
        self.sensors.setdefault(name,sensor)

class hubo_history:

    def __init__(self,filename=None):
        self.states=[]
        self.read_from_file(filename)

    def parse(self,raw_line):
        datalist=re.split('[\t =]+',raw_line)
        #print datalist
        name=datalist[0]
        if mapping.is_ha_joint(name):
            self.states[-1].insert_joint(name,[float(x) for x in datalist[3:12:2]])
        elif re.search('t',datalist[0]):
            try:
                t=float(datalist[1])
                self.states.append(hubo_state(t))
            except ValueError:
                print "time not found in "+datalist[1]

    def read_from_file(self,filename):
        f=open(filename,'r')

        for line in f:
            self.parse(line)

    def slice_states(self,name,field):
        return [s.joints[name][field] for s in self.states]

    def write_joint_pair(self,j0,j1,destname='out.pair'):
        sl0=array(self.slice_states(j0,2))
        sl1=array(self.slice_states(j1,2))
        with open(destname,'w') as f:
            for p1,p2 in zip(sl0,sl1):
                f.write('{},{}\n'.format(p1,p2))


if __name__=='__main__':
    import fnmatch
    import os
    fname=''
    for f in os.listdir('.'):
        if fnmatch.fnmatch(f, sys.argv[1]):
            print f
            fname = f
            break

    import cProfile, openhubo.startup
    pr = cProfile.Profile()
    pr.enable()
    hist=hubo_history(fname)
    pr.disable()
    pr.print_stats('time')

    #LHP=array(hist.slice_states('LHP',2))
    #LHR=array(hist.slice_states('LHR',2))

    #hist.write_joint_pair('LHP','LHR','LHP-LHR.pair')
    #hist.write_joint_pair('RSP','RSR','RSP-RSR.pair')
    #hist.write_joint_pair('RAP','RAR','RAP-RAR.pair')
    #hist.write_joint_pair('LAP','LAR','LAP-LAR.pair')

