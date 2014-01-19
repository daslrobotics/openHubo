from openravepy import CollisionReport
from numpy import zeros, cross, array, pi, cos, sin, mat
import openhubo as oh
from openhubo.comps import Transform
from scipy.spatial import ConvexHull
from scipy.spatial.qhull import QhullError
import numpy as np
from nullspace import nullspace
import time

#TODO declare a vector of peak forces for each contact body?

class ContactCheck:
    def __init__(self,robot, mu=0.2):
        self.robot = robot
        self.env = robot.GetEnv()
        self.links = {}
        self.mu = mu
        self.min_dist=-1

    def insert_link(self,linkname):
        l = robot.GetLink(linkname)
        if l is not None:
            self.links[linkname]=l

    def enable(self):
        for n,l in self.links.items():
            l.Enable(True)

    def disable(self):
        for n,l in self.links.items():
            l.Enable(False)

    def build_cws(self):
        report=CollisionReport()


        self.CWS=[zeros(6)]
        self.positions=[zeros(3)]
        for n,l in self.links.items():

            env.CheckCollision(l,report)
            for c in report.contacts:
                basis = nullspace(mat(c.norm))
                for theta in [0,pi/2,pi,3*pi/2]:
                    w = zeros(6)
                    # arbitrary scale factor
                    vec = cos(theta) * array(basis[:,0])+sin(theta)*array(basis[:,1])
                    #TODO limit force by link
                    w[0:3] = np.squeeze(array(c.norm) + np.squeeze(vec) * self.mu) * 1000
                    w[3:] = cross(c.pos, w[0:3])
                    self.CWS.append(w[0:6])
                    self.positions.append(c.pos)

        try:
            self.hull = ConvexHull(array(self.CWS))
            return True
        except QhullError:
            print "can't build CWS, assuming unstable!"
            return False

    def plot_cones(self,scale = 0.0002):
        normals = array(self.CWS)[:,0:3]
        if len(self.positions):
            plist=np.hstack((array(self.positions),array(self.positions)+normals*scale))
            self.handles = env.drawlinelist(np.reshape(plist,(1,-1)),.5)
        else:
            self.handles = None

    def plot_wrenches(self,scale = 0.0002):
        wrenches = array(self.CWS)[:,3:6]
        if len(self.positions):
            plist=np.hstack((array(self.positions),array(self.positions)+wrenches*scale))
            self.handles = env.drawlinelist(np.reshape(plist,(1,-1)),.5)
        else:
            self.handles = None

    def build_gravity_wrench(self):
        m = oh.find_mass(self.robot)
        g = 9.81

        w_g = zeros(6)

        r = oh.find_com(self.robot)
        m = oh.find_mass(self.robot)

        w_g[0:3] = m*array([0,0,g])
        w_g[3:6] = cross(r,w_g[0:3])

        return w_g

    def build_inertia_wrench(self):
        w_i = zeros(6)
        print "not implemented"
        return w_i

    def check(self, quick=False):
        """Slowest possible way to check for interior, by brute force over all planes."""
        inside = True
        #print "point:", point
        if self.hull is None:
            self.min_dist = -1
            return False
        point = self.build_gravity_wrench()
        self.min_dist=np.infty
        for n,s in enumerate(self.hull.equations):
            res = s[0:6].dot(point[0:6])+s[6]
            dist_internal = abs(min(res,0))
            self.min_dist=min(self.min_dist,dist_internal)
            if res>0:
                inside = False
        print self.min_dist
        return inside

def test_cws(robot):
    check=ContactCheck(robot)
    check.insert_link('leftFoot')
    check.insert_link('rightFoot')
    check.insert_link('rightPalm')
    check.insert_link('leftPalm')
    #print check.build_cws()
    return check

def check_fallen(robot,T_orig,tol=0.03):
    T=robot.GetTransform()
    #print T_orig
    #print T
    if np.linalg.norm(T_orig[:,3] - T[:,3]) > tol:
        return True
    else:
        return False

def make_average_initial_pose(pose,steps=100):
    env=pose.robot.GetEnv()
    env.StopSimulation()
    pose.send(direct=True)
    T=robot.GetTransform()
    for k in xrange(steps):
        env.StepSimulation(oh.TIMESTEP)
        T+=pose.robot.GetTransform()
    T/=(steps+1)
    #Hack to "average" transform
    pose.take_init_pose(trans=T)
    return T

if __name__ == '__main__':

    (env,options)=oh.setup('qtcoin',True)
    env.SetDebugLevel(4)
    #TODO fix hard name here
    options.robotfile='../robots/drchubo/drchubo_v3/robots/drchubo_v3.robot.xml'
    #options.scenefile='../scenes/ladderclimb.env.xml'
    [robot,ctrl,ind,ref,recorder]=oh.load_scene(env,options)

    pose=oh.Pose(robot)
    pose['LF1']=-pi/6
    pose['RF1']=-pi/6
    pose['RF2']=-pi/6
    pose['LSP']=-0.4
    pose['RSP']=-0.4
    leg_tilt=0.16
    pose['LHP']=leg_tilt
    pose['RHP']=leg_tilt
    pose['LAP']=-leg_tilt
    pose['RAP']=-leg_tilt
    pose.send(direct=True)
    env.StartSimulation(oh.TIMESTEP)
    oh.pause(2)
    T = make_average_initial_pose(pose)
    check=test_cws(robot)

    steps=50
    ZMP_check=zeros(steps)
    CWS_check=zeros(steps)
    real_stable=zeros(steps)
    min_dist=zeros(steps)

    tol = 0.005
    upper = 1.0
    lower = 0.0
    test = (lower+upper)/2
    k=0
    while upper-lower > tol:
        env.StopSimulation()
        pose['LSP']=-test
        pose['RSP']=-test
        pose.take_init_pose()
        #FIXME this won't work if the ankle orientation changes (simulation could explode)
        pose.reset(True,True)
        t0=time.time()
        check.build_cws()
        CWS_check[k]=check.check()
        t1=time.time()
        env.StartSimulation(oh.TIMESTEP)
        oh.pause(10)
        real_stable[k]=not(check_fallen(robot,T))
        min_dist[k]=check.min_dist
        print CWS_check[k],real_stable[k],min_dist[k]
        if real_stable[k] == 0:
            upper = test
        else:
            lower = test
        test = (upper+lower)/2
        k+=1


