""" openhubo python package. Most of openhubo's functionality
is available through this module.

Command line usage of openhubo and examples:
    openhubo --example myexample.py

The older python syntax is also usable for launching openhubo scripts:
    python [-i] examples/myexample.py

Make sure to consult the command line options for the openhubo command to see a
complete list of settings:
    openhubo --help

General coding style / glossary:

OpenHubo python functions follow (mostly) the PEP recommended python coding styles:
    functions are named with lowercase and underscores, typically starting with a verb:
        get_timestep(...) or set_robot_color(...) are examples.
    Common "verbs" perform similar actions, within reason:
        get: retrieve and format information from an object (such as an openrave environment or robot)
        set: the opposite of get, set a property/parameter to a value, often with some convenience steps thrown in to simplify where possible.
        apply: Similar to set, but used for properties that might be considered "states", and will change often.
        form: Build a data structure or command using smaller input pieces.
"""

__version__='0.7.1-beta'

import numpy as _np
import matplotlib.pyplot as _plt
import atexit as _atexit
import optparse as _optparse
import os as _os
import fnmatch as _fnmatch
import re as _re
import collections

import openravepy as _rave
from recorder import viewerrecorder as _recorder

#Specific useful functions
from numpy import array,zeros,mat
from time import sleep
from datetime import datetime
from . import mapping
from types import ModuleType
from openravepy import KinBody

TIMESTEP=0.001

class Pose:
    """Easy-to-use wrapper for an array of DOF values for a robot. The Pose class
        behaves like a combination of a dictionary and an array. You can look
        up joints by DOF index, openHubo joint name, or Hubo-ach joint name.

        --Examples--
        1. Create a pose from a robot:
            pose=Pose(robot,[ctrl])
        2. Get / Change a joint value in the local pose:
            pose['REP']
            pose['LHP']=pi/4
        3. Update to the latest robot pose:
            pose.update()
        4. Send a pose to the robot's controller:
            pose.send()
    """

    def build_joint_index_map(self,robot):
        jmap= {j.GetName():j.GetDOFIndex() for j in robot.GetJoints()}
        for (k,v) in mapping.deprecated_names.iteritems():
            if jmap.has_key(v):
                jmap[k]=jmap[v]
        return jmap

    def __init__(self,robot=None,ctrl=None,values=None,useregex=False):
        self.robot=robot
        self.jointmap=self.build_joint_index_map(robot)
        if values is not None:
            #Will throw size exception if values is too short
            self.values=values
        elif robot is not None:
            self.update()
        else:
            self.values=zeros(robot.GetDOF())

        if ctrl:
            self.ctrl=ctrl
        else:
            try:
                self.ctrl=robot.GetController()
            except AttributeError:
                #If using a kinbody, need this hack to get the right robot and ctrl
                self.robot=robot.GetEnv().GetRobot(robot.GetName())
                self.ctrl=self.robot.GetController()

        self.useregex=useregex

    def update(self,newvalues=None):
        """manually assign new values, or poll for new values from robot"""
        if newvalues is None:
            self.values=self.robot.GetDOFValues()
        elif len(newvalues)==len(self.values):
            self.values=array(newvalues)
            #TODO: exception throw here?

    def reset(self,values=None,send=True,vel=False):
        """Quick reset pose back to zero:
            :param send: Additionally set the robot to the reset pose.
            :param vel: Also reset the robot's DOF velocities (WARNING: this
                will cause "hard" update that is non-physical.

        This is a convenience function, consider using "with robot:" as a more robust alternative.
        """

        if values is None:
            values=zeros(self.robot.GetDOF())
        self.values=values

        if send:
            with self.robot.GetEnv():
                self.send()
                if vel:
                    self.robot.SetVelocity([0,0,0],[0,0,0])
                    self.robot.SetDOFVelocities(self.values)

    def to_waypt(self,dt=1,affine=zeros(7)):
        #list constructor does shallow copy here
        waypt =  [float(v) for v in self.values]
        #Add affine pose information if needed
        waypt.extend(affine)
        waypt.append(dt)
        return waypt

    def send(self,direct=False):
        if direct:
            self.robot.SetDOFValues(self.values)
        else:
            self.ctrl.SetDesired(self.values)

    def pretty(self):
        for d, v in enumerate(self.values):
            print '{0} = {1}'.format(
                self.robot.GetJointFromDOFIndex(d).GetName(),v)

    def apply_transform(self,scale=None,offset=None):
        """Apply a shift / scale transformation to the set of joint values. Note the order of operations:
            1) multiply scale factor (also signs using -1), to match a pose to a given sign convention
            2) add offset wrt the SCALED joint values.

        Keep this order in mind as a lot of trouble with trajectories can be traced to messing this up.
        """
        #TODO: Proper exceptions
        if scale is not None:
            self.values*=scale

        if offset is not None:
            self.values+=self.joint_offsets

    def __getitem__(self,key):
        """ Lookup the joint name and return the value"""
        if type(key)==str:
            if self.useregex:
                #Apply a value to multiple joints by regex, slow!
                for k in self.jointmap.keys():
                    if _re.search(key,k):
                        return self.values[self.jointmap[k]]
            else:
                return self.values[self.jointmap[key]]

        if type(key)==slice or type(key)==int:
            return self.values[key]
        if type(key)==KinBody.Joint:
            return self.values[key.GetDOFIndex()]

    def __setitem__(self,key,value):
        """ Lookup the joint name and assign the specified value """
        if type(key)==slice or type(key)==int:
            self.values[key]=value
        elif type(key)==str:
            #TODO: warning for failure
            if self.jointmap.has_key(key):
                self.values[self.jointmap[key]]=value
            elif self.useregex:
                #Apply a value to multiple joints by regex, slow!
                for k in self.jointmap.keys():
                    if _re.search(key,k):
                        self.values[self.jointmap[k]]=value
            else:
                self.values[self.jointmap[key]]=value
        elif type(key)==KinBody.Joint:
            self.values[key.GetDOFIndex()]=value


    def __iadd__(self,other):
        """Add and assign joint values from a pose or array-like object"""
        self.values+=other[:]

    def __isub__(self,other):
        """Subtract and assign joint values from a pose or array-like object"""
        self.values-=other[:]

    def __imul__(self,other):
        """Multiply and assign joint values from a pose or array-like object"""
        if isinstance(other,collections.Iterable):
            self.values*=other[:]
        else:
            self.values*=other

    def __idiv__(self,other):
        """Divide and assign joint values from a pose or array-like object"""
        if isinstance(other,collections.Iterable):
            self.values/=other[:]
        else:
            self.values/=other

    def __add__(self,other):
        """Add and return joint values from a pose or array-like object"""
        return self.values+other[:]

    def __sub__(self,other):
        """Subtract and return joint values from a pose or array-like object"""
        return self.values-other[:]

    def __abs__(self):
        """Return absolute joint values"""
        return abs(self.values)

    def __iter__(self):
        """Iterate over joint values"""
        return iter(self.values)


def set_robot_color(robot,dcolor=[.5,.5,.5],acolor=[.5,.5,.5],trans=0,links=[]):
    """Iterate over a robot's links and set color / transparency."""
    if not len(links):
        links=robot.GetLinks()
    for l in links:
        for g in l.GetGeometries():
            g.SetDiffuseColor(dcolor)
            g.SetAmbientColor(acolor)
            g.SetTransparency(trans)

def get_timestamp(lead='_'):
    """Return a simple formatted timestamp for creating files and such."""
    return lead+datetime.now().strftime("%Y_%m_%d_%H_%M_%S")

def pause(t=-1):
    """ A simple pause function to emulate matlab's pause(t).
    Useful for debugging and program stepping"""
    if t==-1:
        raw_input('Press any key to continue...')
    elif t>=0:
        sleep(t)

def make_name_to_index_converter(robot,autotranslate=True):
    """ A closure to easily convert from a string joint name to the robot's
    actual DOF index.

    Example usage:
        #create function for a robot
        pose=robot.GetDOFValues()
        ind = make_name_to_index_converter(robot)
        #Use the function to find an index in a vector of DOF values
        pose[ind('LHP')]=pi/4
        #This way you don't have to remember the DOF index of a joint to tweak it.
    """
    if autotranslate:
        def convert(name):
            if name is None:
                return None

            j=robot.GetJoint(name)
            if j is None:
                hname=mapping.get_name_from_huboname(name,robot)
                if hname is not None:
                    j=robot.GetJoint(hname)

            if j is not None:
                return j.GetDOFIndex()

            return None
    else:
        def convert(name):
            j=robot.GetJoint(name)
            if j is not None:
                return j.GetDOFIndex()
            else:
                return None
    return convert

def load_scene(env,robotfile=None,scenefile=None,stop=None,physics=True,ghost=False,options=None):
    """ Load files and configure the simulation environment based on arguments and the options structure.
    The returned tuple contains:
        :robot: handle to the created robot
        :ctrl: either trajectorycontroller or idealcontroller depending on physics
        :ind: name-to-joint-index converter
        :ref: handle to visualization "ghost" robot
        :recorder: video recorder python class for quick video dumps
    """

    if not (type(robotfile) is list or type(robotfile) is str):
        _rave.raveLogWarn("Assuming 2nd argument is options structure...")
        options=robotfile

    else:
        (options,__)=get_options()
        options.robotfile=robotfile
        options.scenefile=robotfile
        options.stop=stop
        if physics:
            options._physics='ode'
            options.physicsfile='physics.xml'
        options.ghost=ghost

    return load_scene_from_options(env,options)


def load_scene_from_options(env,options):
    """Load a scene for openhubo based on the options structure (see setup function for detailed options)."""
    if hasattr(options,'physics') and options.physics is False:
        #Kludge since we won't be not using ODE for a while...
        physics=False
    elif hasattr(options,'physics') and options.physics:
        physics=True
    elif options._physics=='ode':
        physics=True
    else:
        physics=False

    vidrecorder=_recorder(env,filename=options.recordfile)
    vidrecorder.videoparams[0:2]=[1024,768]
    vidrecorder.realtime=False

    with env:
        if options.stop or physics:
            #KLUDGE: need to do this for stability reasons, not sure why, probably can be fixed another way
            print "Stopping OpenRAVE simulation to load models and configure..."
            env.StopSimulation()

        if type(options.scenefile) is list:
            for n in options.scenefile:
                env.Load(n)
        elif type(options.scenefile) is str:
            env.Load(options.scenefile)

        #This method ensures that the URI is preserved in the robot
        if options.robotfile is not None:
            robot=env.ReadRobotURI(options.robotfile)
            env.Add(robot)
        else:
            robot=env.GetRobots()[0]
            _rave.raveLogWarn("Assuming robot is {} (id=0)...".format(robot.GetName()))

        robot.SetDOFValues(zeros(robot.GetDOF()))

    #Legacy command mode
    if options.physicsfile==True:
        options.physicsfile='physics.xml'

    with env:
        if physics and not check_physics(env):
            _rave.raveLogInfo('Loading physics parameters from "{}"'.format(options.physicsfile))
            env.Load(options.physicsfile)
        elif not physics:
            env.SetPhysicsEngine(_rave.RaveCreatePhysicsEngine(env,'GenericPhysicsEngine'))
        else:
            #TODO: find a more efficient way to avoid double creation?
            _rave.raveLogInfo("Physics engine already configured, overwriting...")
            env.Load(options.physicsfile)

        if check_physics(env):
            _rave.raveLogInfo('Creating controller for physics simulation')
            controller=_rave.RaveCreateController(env,'trajectorycontroller')
            robot.SetController(controller)
            controller.SendCommand('set gains 150 0 .9 ')
            controller.SendCommand('set radians ')
        else:
            #Just load ideal controller if physics engine is not present
            _rave.raveLogInfo('Physics engine not loaded, using idealcontroller...')
            controller=_rave.RaveCreateController(env,'idealcontroller')
            robot.SetController(controller)

        if options.ghost and options.robotfile:
            ghost_robot=load_ghost(env,options.robotfile,prefix="ghost_")
            if check_physics(env):
                controller.SendCommand("set visrobot " + ghost_robot.GetName())
        else:
            ghost_robot=None

        collisionChecker = _rave.RaveCreateCollisionChecker(env,'pqp')
        if collisionChecker==None:
            collisionChecker = _rave.RaveCreateCollisionChecker(env,'ode')
            _rave.raveLogWarn('Using ODE collision checker since PQP is not available...')
        env.SetCollisionChecker(collisionChecker)

    ind=make_name_to_index_converter(robot)

    if options.atheight is not None:
        align_robot(robot,options.atheight)
        if ghost_robot:
            align_robot(ghost_robot,options.atheight)
    #FIXME: better way to do this?
    global TIMESTEP
    TIMESTEP=get_timestep(env)

    return (robot,controller,ind,ghost_robot,vidrecorder)


def load_ghost(env,robotname,prefix="ref_",color=[.8,.8,.4]):
    """ Create a ghost robot to overlay with an existing robot in the world to show an alternate state."""

    ref_robot=env.ReadRobotURI(robotname)
    ref_robot.SetName(prefix+ref_robot.GetName())
    ref_robot.Enable(False)
    env.Add(ref_robot)
    ref_robot.SetController(_rave.RaveCreateController(env,'mimiccontroller'))
    set_robot_color(ref_robot,color,color,trans=.5)
    return ref_robot


def make_ghost_from_robot(robot,prefix="ref_",color=[.8,.8,.4]):
    """ Not yet implemented """
    pass

def check_physics(env):
    """Helper function to see if physics is currently enabled"""
    return env.GetPhysicsEngine().GetXMLId()!='GenericPhysicsEngine'

def get_timestep(env):
    if check_physics(env):
        return 0.001
    else:
        return 0.005

def align_robot(robot,setheight=0.000,floornormal=[0,0,1]):
    """ Align robot to floor, spaced slightly above"""
    env=robot.GetEnv()
    with env:
        T=robot.GetTransform()
        bb=robot.ComputeAABB()
        bottom_corner=bb.pos()-bb.extents()

        dh=setheight-bottom_corner[2]

        # add height change to robot
        T[2,3] += dh
        robot.SetTransform(T)
        #TODO: reset velocity?


#####################################################################
# Mass functions
#####################################################################

def find_com(robot):
    """
    Find center of mass or robot
    """
    com_trans = _np.array([0.0, 0.0, 0.0])
    mass = 0.0
    for link in robot.GetLinks():
        com_trans += (link.GetGlobalCOM() * link.GetMass())
        mass += link.GetMass()

    com = com_trans / mass
    return com


def find_mass(robot):
    mass = 0
    for link in robot.GetLinks():
        mass = mass + link.GetMass()

    return mass


def plot_contacts(robot,scale=.1):
    env = robot.GetEnv()
    with env:
        # setup the collision checker to return contacts
        checker = env.GetCollisionChecker()
        checker.SetCollisionOptions(_rave.CollisionOptions.Contacts)

        # get first collision
        report = _rave.CollisionReport()
        env.CheckCollision(robot, report=report)
        #_rave.raveLogInfo('{} contacts'.format(len(report.contacts)))
        positions = _np.array([c.pos for c in report.contacts])
        normals = _np.array([c.norm for c in report.contacts])

    if len(positions):
        p_handles = env.plot3(positions, 5, [.7, .3, .3])
        plist=_np.hstack((positions,positions+normals*scale))
        n_handles = env.drawlinelist(_np.reshape(plist,(1,-1)),.5)
        handles=(p_handles,n_handles)
    else:
        handles = None

    return handles

def plot_dirs(robot,scale=.1):
    env = robot.GetEnv()
    manips=robot.GetManipulators()
    trans = _np.array([m.GetEndEffectorTransform()[0:3,3] for m in manips])
    dirs = _np.array([apply_transform(m.GetTransform(),m.GetDirection()) for m in manips])
    if len(dirs):
        p_handles = env.plot3(trans, 5, [.7, .3, .3])
        plist=_np.hstack((trans,trans+dirs*scale))
        n_handles = env.drawlinelist(_np.reshape(plist,(1,-1)),.5)
        handles=(p_handles,n_handles)
    return handles

def apply_transform(T,v):
    newv=mat(T)*mat(_np.hstack((v,0))).T
    return array(newv[0:3]).squeeze()

def plot_body_com(link, handle=None, color=None):
    """ efficiently plot the center of mass of a given link"""

    if color is None:
        color = _np.array([0, 1, 0])
    origin = link.GetGlobalCOM()
    mass = link.GetMass()
    #Fetch environment from robot parent
    env = link.GetParent().GetEnv()
    if handle is None:
        handle = env.plot3(
            points=origin, pointsize=5.0 * mass, colors=color)
    else:
        neworigin = [1, 0, 0, 0]
        neworigin.extend(origin.tolist())
        handle.SetTransform(_rave.matrixFromPose(neworigin))
    return handle


def plot_projected_com(robot):
    proj_com = find_com(robot)
    #assume zero height floor for now
    proj_com[-1] = 0.001

    env = robot.GetEnv()

    point_color = _np.array([0, 1, 1])
    handle = env.plot3(points=proj_com, pointsize=12, colors=point_color)
    return handle


def plot_masses(robot, color=None, com_color=None):
    """Plot spheres at each link center of mass. The volume of each sphere
    corresponds to to the sphere's mass in 10'ths of a kg.
    """
    if color is None:
        color = _np.array([.8, .5, .3])

    if com_color is None:
        com_color = [0., .8, .8]

    handles = []
    for l in robot.GetLinks():
        origin = l.GetGlobalCOM()
        m = l.GetMass()
        handles.append(robot.GetEnv().plot3(origin, m/100., _np.array(color), True))
    handles.append(robot.GetEnv().plot3(find_com(robot), m/100., com_color, True))
    return handles


class ServoPlotter:
    """A simple class to import recorded servo data and plot a specific subset
    of joints. matplotlib.pyplot commands are embedded in the class so you can
    easily customize the plot.
    :param filename: file containing recorded servo data. """

    #Static plotting functions for simplicity

    def __init__(self,filename=None,servolist=[]):
        self.jointdata={}
        self.veldata={}
        self.import_servo_data(filename)
        if len(servolist)>0:
            self.plot(servolist)

    def import_servo_data(self,filename,clearold=False):
        """ Read in servo data from the filename provided."""

        with open(filename,'r') as f:
            gainstring=f.readline().rstrip()
            print gainstring
            servostrings=f.readlines()

        if clearold:
            self.jointdata={}

        for l in servostrings:
            data=l.rstrip().split(' ')
            #Store a dictionary of lists?
            self.jointdata.setdefault(data[0],[float(x) for x in data[1:]])
        self.calc_vel()

    def plot(self,servolist=None):
        if servolist is None:
            servolist=self.jointdata.keys()
        for s in servolist:
            REF='{}_REF'.format(s)
            _plt.plot(self.jointdata[REF],'+',hold=True)
            _plt.plot(self.jointdata[s],hold=True)
            _plt.show()

    def vel_plot(self,servolist=None):
        if servolist is None:
            servolist=self.veldata.keys()
        for s in servolist:
            REF='{}_REF'.format(s)
            _plt.plot(self.veldata[REF],'+',hold=True)
            _plt.plot(self.veldata[s],hold=True)
            _plt.show()

    def calc_vel(self,dt=TIMESTEP):
        for k in self.jointdata.keys():
            self.veldata.setdefault(k,_np.diff(self.jointdata[k])/dt)

#####################################################################
# Robot utility functions
#####################################################################

def set_servo_torquemode(robot,joints,mode='directtorque'):
    """Shortcut function to set a given set of servo joints to the desired control mode"""
    cmd=[mode]
    for j in joints:
        cmd.append(str(j.GetDOFIndex()))

    return robot.GetController().SendCommand(' '.join(cmd))


def _safe_quit():
    """ Exit callback to ensure that openrave closes safely."""
    #Somewhat overkill, try to avoid annoying segfaults
    _rave.raveLogDebug("Destroying Rave Environments...")
    for env in _rave.RaveGetEnvironments():
        env.Destroy()
    #Removed redundant RaveDestroy

def _create_parser(parser=None):
    """Create a new openhubo argument parser, or extend an existing parser
    passed as an argument.  Note that there is currently no conflict checking,
    so make sure your pre-configured parser doesn't conflict with openhubo
    options."""
    if parser is None:
        parser = _optparse.OptionParser(description='OpenHubo: perform experiments with virtual hubo modules.',
                                        usage='usage: %prog [options] script')
    _rave.misc.OpenRAVEGlobalArguments.addOptions(parser)

    scene_options=_optparse.OptionGroup(parser,'Scene Setup',
                                        'Use these options to specify how the simulated world is loaded.')
    #Robot configuration
    parser.set_defaults(robotfile='rlhuboplus.robot.xml')
    scene_options.add_option('--robot', action="store",type='string',dest='robotfile',
                             help='Robot XML file (default=%default)')
    scene_options.add_option('--no-robot', action="store_false",dest='robotfile',
                             help='Do not load a robot from a separate file ')

    parser.set_defaults(scenefile='floor.env.xml')
    scene_options.add_option('--scene', action="store",type='string',dest='scenefile',
                             help='Scene file to load (default=%default)')
    scene_options.add_option('--no-scene', action="store_false",dest='scenefile',
                             help='Do not load an XML scene file')

    scene_options.add_option('--physicsfile', action="store",dest='physicsfile',default='physics.xml',
                             help='Specify a physics engine configuration from XML file (default=%default)')
    scene_options.add_option('--no-physics', action="store_false",dest='physics',default=None,
                             help='Force disable loading of physics engines in openhubo programs')
    scene_options.add_option('--ghost', action="store_true",dest='ghost',default=False,
                             help='Create a ghost robot to show desired vs. actual pose')
    scene_options.add_option('--atheight', action="store",type="float", dest='atheight',default=None,
                             help='Align the robot\'s feet at the given absolute Z height')

    parser.add_option_group(scene_options)
    #Interaction controls
    code_options=_optparse.OptionGroup(parser,'Interaction and Testing','Diagnostic tools and settings for code')
    code_options.add_option('-i','--interpreter', action="store", dest='interpreter',default='ipython',
                            help='Choose the python shell to drop into for interactive mode (default=%default). NOTE: overrides default openrave behavior')
    code_options.add_option('--no-interact', action="store_false",dest='interact',default=True,
                            help='Disable interactive prompt and exit after running')
    code_options.add_option('--no-interactive-imports', action="store_true", dest='noimports',default=False,
                            help='Disable bulk imports for interactive prompt (useful for debugging import errors)')
    code_options.add_option("-p","--profile", action="store_true",
                            dest="profile", default=False,
                            help="Use python profiler for analysis")
    code_options.add_option('--testsuite', action="store", dest='testsuite',default=None,
                            help='(Not yet implemented) run the openhubo testsuite')
    code_options.add_option("--pdb", action="store_true",
                            dest="pdb", default=False,
                            help="Use python debugger")

    parser.add_option_group(code_options)

    parser.add_option('--example', action="store",type='string',dest='example',default=None,
                      help='Run a python example from the examples folder')
    parser.add_option('--record', action="store",dest='recordfile',default=None,
                      help='Enable video recording to the given file name (requires script commands to start and stop)')
    parser.add_option('--stop-simulation', action="store_true", dest='stop',default=None,
                      help='Stop the simulation during scene / robot loading')
    parser.add_option('--no-stop-simulation', action="store_false", dest='stop',
                      help='Do not stop the simulation during scene / robot loading')
    parser.add_option('--video-capture-file', action="store", dest='recordfile',default=None,
                      help='Specify a video file for the video recorder to capture to.')
    return parser
    #TODO: add callback to clean up "None"'s

def setup(viewername=None,create=True,parser=None):
    """ Setup openhubo environment and viewer when run from the command line.
    :param viewername: Name of viewer plugin to use (defaults to no viewer)
    :param create: If true, set up an environment.
    :param parser: Pass in a pre-defined option parser for custom flags and options.
    """
    (options,leftargs)=get_options(viewername,parser)

    if create:
        env=_rave.Environment()
        _atexit.register(_safe_quit)
        try:
            _rave.misc.OpenRAVEGlobalArguments.parseEnvironment(options,env)
        except TypeError as e:
            if isinstance(viewername,ModuleType):
                print "Running nosetests?"
            else:
                raise e

    else:
        env=None

    return (env,options)

def get_options(viewername=None,parser=None):
    """Parse options from command line, extending an existing parser or
    creating a new one if none is specified."""

    parser=_create_parser(parser)
    (options, leftargs) = parser.parse_args()

    if viewername:
        #Overwrite command line option with explicit argument
        options._viewer=viewername

    if options.robotfile=="none" or options.robotfile=="None":
        #possible command line fake for "none"
        options.robotfile=None

    if options.scenefile=="none" or options.scenefile=="None":
        #possible command line fake for "none"
        options.scenefile=None

    return (options,leftargs)

def get_root_dir():
    return _os.environ['OPENHUBO_DIR']

def find_files(directory, pattern):
    for root, dirs, files in _os.walk(directory):
        for basename in files:
            if _fnmatch.fnmatch(basename, pattern):
                filename = _os.path.join(root, basename)
                yield filename

def find(rawname, path=None):
    (fpath,fname)=_os.path.split(rawname)
    #TODO: make better assumptions about name
    if not path:
        path=get_root_dir()
    for root, dirs, files in _os.walk(path+'/'+fpath):
        if fname in files:
            return _os.path.join(root, fname)

def list_robots(pattern='*.robot.xml',directory='robots'):
    filenames=[]
    for root, dirs, files in _os.walk(directory):
        for basename in files:
            if _fnmatch.fnmatch(basename, pattern):
                filenames.append( _os.path.join(root, basename))
    return filenames

