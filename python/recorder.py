#!/usr/bin/env python

import tab
from openravepy import *
from numpy import *
import time
from copy import copy

class viewerrecorder:

    def __init__(self,env,copyfrom=None):
        if copyfrom==None:
            self.filename='recorded_video'
            self.codec=13
            self.videoparams=[640,480,30]
            self.realtime=True
        else:
            self.filename=copyfrom.filename
            self.codec=copyfrom.codec
            self.videoparams=copyfrom.videoparams
            self.realtime=copyfrom.realtime
        self.env=env
        self.module=RaveCreateModule(env,'viewerrecorder')
        self.env.AddModule(self.module,'')
        #TODO: garbage collection for duplciate modules created this way

    def start(self):
        vidstring=' '.join(['{}'.format(k) for k in self.videoparams])
        if self.realtime==True:
            timing='realtime'
        else:
            timing='simtime'
        viewer=self.env.GetViewer().GetName()
        cmd='Start ' + vidstring + ' timing ' + timing + ' filename ' + self.filename +'\n viewer ' + viewer + '\n'
        print cmd
        self.module.SendCommand(cmd)

    def stop(self):
        self.module.SendCommand('Stop')

if __name__ == '__main__':
    
    file_env = 'scenes/simpleFloor.env.xml'

    env = Environment()
    env.SetViewer('qtcoin')
    env.SetDebugLevel(3)
    env.Load(file_env)
    env.StartSimulation(timestep=0.001)


    recorder=viewerrecorder(env)
    recorder.start()
    time.sleep(6)
    recorder.stop()
