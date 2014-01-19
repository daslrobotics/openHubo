from openravepy import CollisionReport
from numpy import zeros, cross, array, pi, cos, sin, mat
import openhubo as oh
from openhubo.comps import Transform
from scipy.spatial import ConvexHull
from scipy.spatial.qhull import QhullError
import numpy as np
from nullspace import nullspace

#TODO declare a vector of peak forces for each contact body?

class ContactCheck:
    def __init__(self,robot, mu=0.2):
        self.robot = robot
        self.env = robot.GetEnv()
        self.links = {}
        self.mu = mu

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
        #TODO exception?
            return False
        point = self.build_gravity_wrench()
        self.distances = zeros(np.size(self.hull.equations,0))
        for n,s in enumerate(self.hull.equations):
            res = s[0:6].dot(point[0:6])+s[6]
            self.distances[n]=res
            if res>0:
                inside = False
                if quick:
                    return inside
                #print "point is outside of face {}".format(n)
            #else:
                #print "point inside of face P{}".format(n)
        self.min_dist = min(self.distances)
        return inside

def test_cws(robot):
    check=ContactCheck(robot)
    check.insert_link('leftFoot')
    check.insert_link('rightFoot')
    check.insert_link('rightPalm')
    check.insert_link('leftPalm')
    print check.build_cws()
    return check


class ContactSphere:
    suffix=0

    def __init__(self, robot, transform,linkname):
        self.robot = robot
        self.link = robot.GetLink(linkname)
        self.transform = Transform(transform)
        env = robot.GetEnv()
        env.Load('contactsphere.kinbody.xml')
        self.sphere = env.GetKinBody('contactsphere')
        self.sphere.SetName('contactsphere_{}'.format(ContactSphere.suffix))
        ContactSphere.suffix+=1

    def update(self):
        global_transform = Transform(self.link.GetTransform()) * self.transform
        self.sphere.SetTransform(array(global_transform))

    def check(self):
        report = CollisionReport()
        self.sphere.Enable(True)
        env = self.robot.GetEnv()
        res = env.CheckCollision(self.sphere,report=report)
        self.sphere.Enable(False)
        return res

class ContactSphereSet:
    def __init__(self):
        self.spheres = []

    def append(self,sphere):
        self.spheres.append(sphere)

    def update(self):
        for s in self.spheres:
            s.update()

    def check(self):
        contacts = [s.check() for s in self.spheres]
        return contacts

class SphereCheck:
    def __init__(self,robot):
        self.robot = robot
        self.contactsets={}
        self.activecontacts = {}

    def insert_contacts(self, linkname, contactset):
        self.contactsets.setdefault(linkname,contactset)

    def build_active_sets(self):
        for l,s in self.contactsets.items():
            s.update()
            self.activecontacts.setdefault(l,s.check())

f_standoff = 0.004
def create_right_foot(robot):
    rightFootSpheres=ContactSphereSet()
    rightFootSpheres.append(ContactSphere(robot,Transform(trans=[0.063,0.058,-f_standoff]),'rightFoot'))
    rightFootSpheres.append(ContactSphere(robot,Transform(trans=[0.063,-0.065,-f_standoff]),'rightFoot'))
    rightFootSpheres.append(ContactSphere(robot,Transform(trans=[-0.133,0.058,-f_standoff]),'rightFoot'))
    rightFootSpheres.append(ContactSphere(robot,Transform(trans=[-0.133,-0.065,-f_standoff]),'rightFoot'))
    return rightFootSpheres

def create_left_foot(robot):
    leftFootSpheres=ContactSphereSet()
    leftFootSpheres.append(ContactSphere(robot,Transform(trans=[0.063,-0.058,-f_standoff]),'leftFoot'))
    leftFootSpheres.append(ContactSphere(robot,Transform(trans=[0.063,0.065,-f_standoff]),'leftFoot'))
    leftFootSpheres.append(ContactSphere(robot,Transform(trans=[-0.133,-0.058,-f_standoff]),'leftFoot'))
    leftFootSpheres.append(ContactSphere(robot,Transform(trans=[-0.133,0.065,-f_standoff]),'leftFoot'))
    return leftFootSpheres

h_standoff = 0.001

def create_right_palm(robot):
    rightPalmSpheres=ContactSphereSet()
    rightPalmSpheres.append(ContactSphere(robot,Transform(trans=[0.011,h_standoff,0.031]),'rightPalm'))
    rightPalmSpheres.append(ContactSphere(robot,Transform(trans=[0.011,h_standoff,-0.031]),'rightPalm'))
    rightPalmSpheres.append(ContactSphere(robot,Transform(trans=[-0.011,h_standoff,0.031]),'rightPalm'))
    rightPalmSpheres.append(ContactSphere(robot,Transform(trans=[-0.011,h_standoff,-0.031]),'rightPalm'))
    return rightPalmSpheres

def create_left_palm(robot):
    leftPalmSpheres=ContactSphereSet()
    leftPalmSpheres.append(ContactSphere(robot,Transform(trans=[0.011,-h_standoff,0.031]),'leftPalm'))
    leftPalmSpheres.append(ContactSphere(robot,Transform(trans=[0.011,-h_standoff,-0.031]),'leftPalm'))
    leftPalmSpheres.append(ContactSphere(robot,Transform(trans=[-0.011,-h_standoff,0.031]),'leftPalm'))
    leftPalmSpheres.append(ContactSphere(robot,Transform(trans=[-0.011,-h_standoff,-0.031]),'leftPalm'))
    return leftPalmSpheres


def check_fallen(robot,T_orig,tol=0.03):
    T=robot.GetTransform()
    if T_orig[2,3] - T[2,3] > tol:
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
    pose['LSP']=-0.3
    pose['RSP']=-0.3
    leg_tilt=0.12
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
    fallen=zeros(steps)
    min_dist=zeros(steps)

    for k in xrange(steps):
        env.StopSimulation()
        pose['LSP']-=0.02
        pose['RSP']-=0.02
        pose.take_init_pose()
        #FIXME this won't work if the ankle orientation changes (simulation could explode)
        pose.reset(True,True)
        CWS_check[k]=check.build_cws()
        env.StartSimulation(oh.TIMESTEP)
        oh.pause(8)
        fallen[k]=check_fallen(robot,T)
        min_dist[k]=check.min_dist
        print CWS_check[k],fallen[k],min_dist[k]

