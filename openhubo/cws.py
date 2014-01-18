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

    def perform_cws(self):
        report=CollisionReport()

        m = oh.find_mass(self.robot)
        g = 9.81

        self.CWS=[array([0,0,0,0,0])]
        for n,l in self.links.items():

            env.CheckCollision(l,report)
            for c in report.contacts:
                basis = nullspace(mat(c.norm))
                for theta in [0,pi/2,pi,3*pi/2]:
                    w = zeros(6)
                    # arbitrary scale factor
                    vec = cos(theta) * array(basis[:,0])+sin(theta)*array(basis[:,1])
                    w[0:3] = np.squeeze(array(c.norm) + np.squeeze(vec) * self.mu) * 2 * m*g
                    w[3:] = cross(c.pos, w[0:3])
                    self.CWS.append(w[0:5])

        w_g = zeros(6)

        r = oh.find_com(self.robot)
        m = oh.find_mass(self.robot)

        w_g[0:3] = m*array([0,0,g])
        w_g[3:6] = cross(r,w_g[0:3])

        #TODO evaluate convex hull of CWS here
        try:
            self.hull = ConvexHull(array(self.CWS))
        except QhullError:
            print "can't build CWS, assuming unstable!"
            return False
        inside = check_interior(self.hull,w_g[0:5])

        return inside

def check_interior(hull, point, thorough=False):
    """Slowest possible way to check for interior, by brute force over all planes."""

    inside = True
    for n,s in enumerate(hull.equations):
        res = s[0:5].dot(point[0:5])+s[5]
        if res>0:
            inside = False
            if thorough:
                return inside
            #print "point is outside of face {}".format(n)
        #else:
            #print "point inside of face P{}".format(n)

    return inside

def test_cws(robot):
    T=robot.GetTransform()
    T[2,3]-=.0015
    robot.SetTransform(T)
    print "Move robot to colliding position"
    oh.pause()
    check=ContactCheck(robot)
    check.insert_link('leftFoot')
    check.insert_link('leftPalm')
    check.insert_link('rightFoot')
    check.insert_link('rightPalm')
    print check.perform_cws()
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

if __name__ == '__main__':

    (env,options)=oh.setup('qtcoin',True)
    env.SetDebugLevel(4)
    #TODO fix hard name here
    options.robotfile='../robots/drchubo/drchubo_v3/robots/drchubo_v3.robot.xml'
    options.scenefile='../scenes/ladderclimb.env.xml'
    [robot,ctrl,ind,ref,recorder]=oh.load_scene(env,options)


    #check = CWSCheck(robot)
    #check.insert_contacts('rightFoot', create_right_foot(robot))
    #check.insert_contacts('leftFoot', create_left_foot(robot))
    #check.insert_contacts('rightPalm', create_right_palm(robot))
    #check.insert_contacts('leftPalm', create_left_palm(robot))
    oh.pause()
    env.StartSimulation(oh.TIMESTEP)
    pose=oh.Pose(robot)
    pose['LF1']=-pi/3
    pose['RF1']=-pi/3
    pose['RF2']=-pi/3
    pose.send()
    #check.build_active_sets()

    check = test_cws(robot)

    for k in xrange(100):
        oh.pause(0.5)
        env.StopSimulation()
        print check.perform_cws()
        env.StartSimulation(oh.TIMESTEP)
