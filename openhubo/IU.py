import fnmatch as _fnmatch
import os as _os
import pickle
import time

from numpy import pi,cos,sin,zeros,ones,array
import numpy as _np
#Include IU trajectory format here for convenience
from .trajectory import IUTrajectory
from openravepy.openravepy_int import Sensor
import openhubo as _oh

## Ladder Format
#0.05		#ladder-stringer-width
#3			#ladder-stringer-height
#0.05		#ladder-stringer-thickness
#0.6			#ladder-rung-width
#0.05		#ladder-rung-height
#0.10		#ladder-rung-thickness
#65			#ladder-angle
#0.20		#ladder-rung-space
#0.025		#ladder-stringer-radius
#20			#ladder-stringer-nface
#0.03		#ladder-rung-radius
#20			#ladder-rung-nface
#0			#1-rect--0-cir-rung-cross-section
#1			#1-rect--0-cir-stringer-cross-section

class IULadderGenerator:
    """ A ladder-generating class for IU ladder-climbing planner"""
    def print_parameters(self):
        for k,v in self.parameters.iteritems():
            print '{}={}'.format(k,v)

    def __init__(self,paramfile=None,write=False):
        self.parameters={}
        if paramfile is not None:
            self.load_parameters(paramfile)
            if write:
                self.make_ladder()
                self.make_ladder_env()

    def load_parameters(self,paramfile):
        filename=_oh.find(paramfile)
        with open(filename,'r') as file_read:
            #line 1
            stringer_width=0
            line = file_read.readline()
            counter=0;
            length =len(line)
            for parser in range(0, length):
                if line[parser]=='\t':
                    counter=parser
            if not line:
                print "stringer width Missing"
            else:
                stringer_width=float(line[0:counter-1])/2
                print "stringer_width   "+str(stringer_width)
            #line 2
            line = file_read.readline()
            counter=0;
            length =len(line)
            for parser in range(0, length):
                if line[parser]=='\t':
                    counter=parser
            stringer_height=0
            if not line:
                print "stringer height Missing"
            else:
                stringer_height=float(line[0:counter-1])/2
                print "stringer_height   "+str(stringer_height)


            #line 3
            line = file_read.readline()
            counter=0;
            length =len(line)
            for parser in range(0, length):
                if line[parser]=='\t':
                    counter=parser
            stringer_thickness=0
            if not line:
                print "stringer thickness Missing"
            else:
                stringer_thickness=float(line[0:counter-1])/2
                print "stringer_thickness   "+str(stringer_thickness)

            #line 4
            line = file_read.readline()
            counter=0;
            length =len(line)
            for parser in range(0, length):
                if line[parser]=='\t':
                    counter=parser
            rung_width=0
            if not line:
                print "rung width Missing"
            else:
                rung_width=float(line[0:counter-1])/2
                print "rung_width   "+str(rung_width)

            #line 5
            line = file_read.readline()
            counter=0;
            length =len(line)
            for parser in range(0, length):
                if line[parser]=='\t':
                    counter=parser
            rung_height=0
            if not line:
                print "rung height Missing"
            else:
                rung_height=float(line[0:counter-1])/2
                print "rung_height   "+str(rung_height)

            #line 6
            line = file_read.readline()
            counter=0;
            length =len(line)
            for parser in range(0, length):
                if line[parser]=='\t':
                    counter=parser
            rung_thickness=0
            if not line:
                print "rung thickness Missing"
            else:
                rung_thickness=float(line[0:counter-1])/2

            #line 7
            line = file_read.readline()
            counter=0;
            length =len(line)
            for parser in range(0, length):
                if line[parser]=='\t':
                    counter=parser
            ladder_angle=0
            if not line:
                print "ladder angle Missing"
            else:
                ladder_angle=float(line[0:counter-1])

            #line 8
            line = file_read.readline()
            counter=0;
            length =len(line)
            for parser in range(0, length):
                if line[parser]=='\t':
                    counter=parser
            rung_space=0
            if not line:
                print "rung space Missing"
            else:
                rung_space=float(line[0:counter-1])


            #line 9
            line = file_read.readline()
            counter=0;
            length =len(line)
            for parser in range(0, length):
                if line[parser]=='\t':
                    counter=parser
            stringer_radius=0
            if not line:
                print "stringer_radius Missing"
            else:
                print line[0:counter-1]
                stringer_radius=float(line[0:counter-1])
            print 'stringer radius is  ' +str(stringer_radius)

            #line 10
            line = file_read.readline()
            counter=0;
            length =len(line)
            for parser in range(0, length):
                if line[parser]=='\t':
                    counter=parser
            stringer_nface=0
            if not line:
                print "stringer_nface Missing"
            else:
                stringer_nface=float(line[0:counter-1])

            #line 11
            line = file_read.readline()
            counter=0;
            length =len(line)
            for parser in range(0, length):
                if line[parser]=='\t':
                    counter=parser
            rung_radius=0
            if not line:
                print "rung_radius Missing"
            else:
                rung_radius=float(line[0:counter-1])


            #line 12
            line = file_read.readline()
            counter=0;
            length =len(line)
            for parser in range(0, length):
                if line[parser]=='\t':
                    counter=parser
            rung_nface=0
            if not line:
                print "rung_nface Missing"
            else:
                rung_nface=float(line[0:counter-1])


            #line 13
            line = file_read.readline()
            counter=0;
            length =len(line)
            for parser in range(0, length):
                if line[parser]=='\t':
                    counter=parser
            rung_shape=0
            if not line:
                print "rung shape Missing"
            else:
                rung_shape=float(line[0:counter-1])


            #line 14
            line = file_read.readline()
            counter=0;
            length =len(line)
            for parser in range(0, length):
                if line[parser]=='\t':
                    counter=parser
            stringer_shape=0
            if not line:
                print "stringer shape Missing"
            else:
                stringer_shape=float(line[0:counter-1])

            rungs=0
            if stringer_shape==1:
                rungs=int(2*stringer_height/rung_space)
            else:
                rungs=int(stringer_nface/rung_space)

        self.parameters.setdefault('rungs',rungs)
        self.parameters.setdefault('stringer_width',stringer_width)
        self.parameters.setdefault('stringer_height',stringer_height)
        self.parameters.setdefault('stringer_radius',stringer_radius)
        self.parameters.setdefault('stringer_thickness',stringer_thickness)
        self.parameters.setdefault('stringer_nface',stringer_nface)
        self.parameters.setdefault('stringer_shape',stringer_shape)
        self.parameters.setdefault('rung_width',rung_width)
        self.parameters.setdefault('rung_height',rung_height)
        self.parameters.setdefault('rung_thickness',rung_thickness)
        self.parameters.setdefault('rung_space',rung_space)
        self.parameters.setdefault('rung_nface',rung_nface)
        self.parameters.setdefault('rung_radius',rung_radius)
        self.parameters.setdefault('rung_shape',rung_shape)
        self.parameters.setdefault('ladder_angle',ladder_angle)

        print 'Parsing done'
        paramname=paramfile.split('/')[-1]
        name_tokens=paramname.split('.')
        if len(name_tokens)>1:
            outname='.'.join(name_tokens[:-1])
        else:
            outname=name_tokens[-1]

        self.scenefile=outname+'.env.xml'
        self.name=outname+'.kinbody.xml'

    def make_ladder(self,paramfile=None,usecached=False):
        # Read the file to obtain time steps and the total time
        if test_filename_exist(self.scenefile) and test_filename_exist(self.name) and usecached:
            #early abort if file exists
            print  self.name + " found! Using cached..."
            return self.name
        elif paramfile:
            self.load_parameters(paramfile)

        params = self.parameters
        outstrings=[]

        outstrings.append('<KinBody name="ladder">')
        outstrings.append('	    	<Body name="Base" type="static">')
        outstrings.append('	      		<Translation>0.0  0.0  0.0</Translation>')
        outstrings.append('')

        ## Writing the Stringers
        # Case cylinder
        if params['stringer_shape']==0:
            outstrings.append(' 			<!-- Left Vertical-->')
            outstrings.append('   			<Geom type="cylinder">')
            outstrings.append('				<radius>'+str(params['stringer_radius'])+'</radius>')
            outstrings.append('				<height>'+str(params['stringer_height'])+'</height>')
            outstrings.append('      			<Translation> ' + str(-params['rung_width']-10*params['stringer_width']/2)+' 0 '+str(params['stringer_nface']/2) +'</Translation>')
            outstrings.append('        			<RotationAxis>1 0 0 90</RotationAxis>')
            outstrings.append('    			</Geom>')
            outstrings.append('')

            outstrings.append(' 			<!-- Right Vertical-->')
            outstrings.append('   			<Geom type="cylinder">')
            outstrings.append('				<radius>'+str(params['stringer_radius'])+'</radius>')
            outstrings.append('				<height>'+str(params['stringer_height'])+'</height>')
            outstrings.append('      			<Translation> ' + str(params['rung_width']+10*params['stringer_width']/2)+' 0 '+str(params['stringer_nface']/2) +'</Translation>')
            outstrings.append('        			<RotationAxis>1 0 0 90</RotationAxis>')
            outstrings.append('    			</Geom>')
            outstrings.append('')

        ## Case cuboid
        else:
            outstrings.append(' 			<!-- Left Vertical-->')
            outstrings.append('			<Geom type="box"> ')
            outstrings.append('   				<!-- extents of the box - half width, height, length-->')
            outstrings.append('      			<Extents>'+str(params['stringer_width'])+' '+str(params['stringer_thickness'])+' '+str(params['stringer_height'])+' </Extents>')
            if params['rung_shape']==1:
                outstrings.append('      			<Translation>' + str(-params['rung_width']-10*params['stringer_width']/2)+' 0 '+str(params['stringer_height'])+' </Translation>')
            else:
                outstrings.append('      			<Translation>' + str(-params['rung_width']-10*params['stringer_width']/2)+' 0 '+str(params['stringer_height'])+' </Translation>')
            outstrings.append('			</Geom>')
            outstrings.append('')
            outstrings.append(' 			<!-- Right Vertical-->')
            outstrings.append('			<Geom type="box"> ')
            outstrings.append('   				<!-- extents of the box - half width, height, length-->')
            outstrings.append('      			<Extents>'+str(params['stringer_width'])+' '+str(params['stringer_thickness'])+' '+str(params['stringer_height'])+' </Extents>')
            if params['rung_shape']==1:
                outstrings.append('      			<Translation>' + str(params['rung_width']+10*params['stringer_width']/2)+' 0 '+str(params['stringer_height'])+' </Translation>')
            else:
                outstrings.append('      			<Translation>' + str(params['rung_width']+10*params['stringer_width']/2)+' 0 '+str(params['stringer_height'])+' </Translation>')
            outstrings.append('			</Geom>')


        ## RUNGS
        ## case rect rungs
        if params['rung_shape']==1:
            for i in range(1,params['rungs']+1):
                outstrings.append('')
                outstrings.append('   			<Geom type="box">')
                outstrings.append('      			<Extents>'+str(params['rung_width']+10*params['stringer_width'])+' '+str(params['rung_thickness'])+ ' '+str(params['rung_height'])+' </Extents>')
                outstrings.append('      			<Translation>0 0 '+str(i*params['rung_space'])+'</Translation>')
                outstrings.append('    			</Geom>')
                outstrings.append('')

        ## case cylinder params['rungs
        else:
            for i in range(1,params['rungs']+1):
                outstrings.append('')
                outstrings.append('   			<Geom type="cylinder">')
                outstrings.append('				<radius>'+str(params['rung_radius'])+'</radius>')
                outstrings.append('				<height>'+str(2*params['rung_width']+10*params['stringer_width'])+'</height>')
                outstrings.append('      				<Translation>0 0 '+str(i*params['rung_space'])+'</Translation>')
                outstrings.append('        			<RotationAxis> 0 0 1 90</RotationAxis>')
                outstrings.append('    			</Geom>')
                outstrings.append('')

        ## End file
        outstrings.append('		</Body>')
        outstrings.append('</KinBody>')

        with open(self.name, 'w') as f:
            f.write('\n'.join(outstrings))
        return self.name

    @staticmethod
    def process_paramname(paramfile):
        paramname=paramfile.split('/')[-1]
        name_tokens=paramname.split('.')
        if len(name_tokens)>1:
            outname='.'.join(name_tokens[:-1])
        else:
            outname=name_tokens[-1]

        scenefile=outname+'.env.xml'
        name=outname+'.kinbody.xml'
        return (scenefile,name)

    def make_ladder_env(self,paramfile=None,usecached=False):
        if paramfile:
            (scenefile,name)=self.process_paramname(paramfile)
            self.scenefile=scenefile
            self.name=name
        else:
            scenefile=self.scenefile
            name=self.name

        #Kludgy interpretation of posix path here, may choke on special chars
        if usecached and test_filename_exist(scenefile) and test_filename_exist(name):
            #early abort if file exists
            print scenefile + " found! Using cached"
            return scenefile

        outstrings=[]
        outstrings.append('<Environment>')
        outstrings.append('  <camtrans>0.030207 -0.889688 1.368581</camtrans>')
        outstrings.append('  <camrotationaxis>-0.999979 0.003834 -0.005172 109.126254</camrotationaxis>')
        outstrings.append('  <camfocal>1.913651</camfocal>')
        outstrings.append('  <bkgndcolor>0.0 0.0 0.</bkgndcolor>')
        outstrings.append('')
        outstrings.append('  <KinBody file="{}">'.format(name))
        t_y=2.*cos(self.parameters['ladder_angle']*pi/180)*self.parameters['stringer_height']
        outstrings.append('    <Translation>0 {} 0</Translation>'.format(t_y))
        outstrings.append('    <RotationAxis>1 0 0 '+str(90-self.parameters['ladder_angle'])+'</RotationAxis>')
        outstrings.append('  </KinBody>')
        outstrings.append('')
        outstrings.append('  <KinBody name="floor">')
        outstrings.append('    <Body type="static">')
        outstrings.append('      <Geom type="box">')
        outstrings.append('        <extents>5 5 1</extents>')
        outstrings.append('        <diffuseColor>0.6 0.6 0.6</diffuseColor>')
        outstrings.append('        <ambientColor>0.6 0.6 0.6</ambientColor>')
        outstrings.append('	      <Translation>0 4 -1</Translation>')
        outstrings.append('      </Geom>')
        outstrings.append('    </Body>')
        outstrings.append('  </KinBody>')
        outstrings.append('')
        outstrings.append('  <KinBody name="wall">')
        outstrings.append('    <Body type="static">')
        outstrings.append('      <Geom type="box">')
        outstrings.append('          <extents>5 1 5</extents>')
        outstrings.append('          <diffuseColor>.9 .1 .6</diffuseColor>')
        outstrings.append('      	<ambientColor>0.6 0.6 0.6</ambientColor>')
        outstrings.append('	        <Translation>0 -1.1 5</Translation>')
        outstrings.append('      </Geom>')
        outstrings.append('    </Body>')
        outstrings.append('  </KinBody>')
        outstrings.append('')
        outstrings.append('</Environment>')
        with open(scenefile,'w') as f:
            f.write('\n'.join(outstrings))

def test_filename_exist(filename,spath='.'):
    for f in _os.listdir(spath):
        if _fnmatch.fnmatch(f, filename):
            return True
    return False

class EffectorLog:
    """A simple logger for a manually-stepped simulation to store End Effector pose data
    data efficiently."""
    def __init__(self,loglen=10,bodies=[]):
        self.width=1
        self.loglen=loglen
        self.bodies=[]
        self.count=0
        self.env=bodies[0].GetParent().GetEnv()
        for b in bodies:
            self.width+=3
            self.bodies.append(b)
        self.robot=self.bodies[0].GetParent()
        self.data=zeros((loglen,self.width))
        self.com=zeros((loglen,3))
        #Data structure is 1 col of time, 6s columns of sensor data

    def record(self,time=None):
        if not time:
            self.data[self.count,0]=self.env.GetSimulationTime()
        else:
            self.data[self.count,0]=time
        for k in range(len(self.bodies)):
            #COM is coincident with body frame due to ODE restrictions
            com=self.bodies[k].GetGlobalCOM()
            c0=1+k*3
            c1=1+3*(k+1)
            self.data[self.count,c0:c1]=(com)
            self.com[self.count,:]=_oh.find(self.robot)
        self.count+=1

    def save(self,filename):
        names=[]
        for b in self.bodies:
            names.append(b.GetName())
        robotname=self.bodies[0].GetParent().GetName()
        with open(filename,'w') as f:
            #TODO: save robot hash?
            pickle.dump([self.data[:self.count,:],self.com[:self.count,:],names,robotname],f)

    def load_scene(self,filename):
        with open(filename,'r') as f:
            self.data=pickle.load(f)
            self.count=self.data.shape[0]
    def lookup(self,name,components):
        if name=='time':
            return self.data[:self.count,0]
        for k in range(len(self.sensors)):
            if self.sensors[k].GetName()==name:
                c0=1+k*3
                return self.data[:self.count,array(components)+c0]

class ForceLog:
    """A simple logger for a manually-stepped simulation to store Force/Torque
    data efficiently."""
    def __init__(self,env,loglen=10,sensors=[]):
        """Initialize a force logger. Note that you need an openrave environment here."""
        self.width=1
        self.loglen=loglen
        self.sensors=[]
        self.count=0
        self.env=env
        self.env=sensors[0].GetAttachingLink().GetParent().GetEnv()
        for s in sensors:
            if type(s.GetData()) is Sensor.Force6DSensorData:
                self.width+=6
                self.sensors.append(s)
        self.data=zeros((loglen,self.width))
        #Data structure is 1 col of time, 6s columns of sensor data

    def setup(self,histlen):
        for s in self.sensors:
            if type(s.GetData()) is Sensor.Force6DSensorData:
                FT=s.GetSensor()
                FT.Configure(Sensor.ConfigureCommand.PowerOff)
                time.sleep(0.1)
                FT.SendCommand('histlen {}'.format(histlen))
                time.sleep(0.1)
                FT.Configure(Sensor.ConfigureCommand.PowerOn)

    def record(self,time=None):
        """Log a force sensor state."""
        with self.env:
            if not time:
                self.data[self.count,0]=self.env.GetSimulationTime()
            else:
                self.data[self.count,0]=time.time()
            for k in range(len(self.sensors)):
                force=self.sensors[k].GetData().force
                torque=self.sensors[k].GetData().torque
                c0=1+k*6
                c1=1+6*(k+1)
                self.data[self.count,c0:c1]=_np.hstack((force,torque))
        self.count+=1

    def save(self,filename):
        names=[]
        for s in self.sensors:
            names.append(s.GetName())
        robotname=self.sensors[0].GetAttachingLink().GetParent().GetName()
        with open(filename,'w') as f:
            #TODO: save robot hash?
            pickle.dump([self.data[:self.count,:],names,robotname],f)

    def load(self,filename):
        with open(filename,'r') as f:
            self.data=pickle.load(f)
            self.count=self.data.shape[0]
    def lookup(self,name,components):
        if name=='time':
            return self.data[:self.count,0]
        for k in range(len(self.sensors)):
            if self.sensors[k].GetName()==name:
                return self.data[self.count,array(components)+1+6*k]
