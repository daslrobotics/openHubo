from openravepy import CollisionReport
from numpy import zeros, cross, array, pi
import openhubo as oh
from openhubo.comps import Transform
from scipy.spatial import ConvexHull
import numpy as np

#TODO declare a vector of peak forces for each contact body?
def perform_cws(robot,links):
    env=robot.GetEnv()
    report=CollisionReport()

    m = oh.find_mass(robot)
    g = 9.81

    CWS=[array([0,0,0,0,0])]
    for l in links:
        link = robot.GetLink(l)
        env.CheckCollision(link,report)
        for c in report.contacts:
            for theta in [0,pi/2,pi,3*pi/2]:
                w = zeros(6)
                # arbitrary scale factor
                w[0:3] = (c.norm+array([np.cos(theta),np.sin(theta),0])*.1) * 2 *m*g
                w[3:] = cross(c.pos, w[0:3])
                CWS.append(w[0:5])

    w_g = zeros(6)

    r = oh.find_com(robot)
    m = oh.find_mass(robot)

    w_g[0:3] = m*array([0,0,g])
    w_g[3:6] = cross(r,w_g[0:3])

    #TODO evaluate convex hull of CWS here
    hull = ConvexHull(array(CWS))

    inside = check_interior(hull,w_g[0:5])

    #TODO reduce CWS with simplified contacts
    return (inside, hull,CWS, report)

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
    stable, hull, CWS, report = perform_cws(robot,['leftFoot','rightFoot','leftPalm','rightPalm'])
    return stable

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

class CWSCheck:
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


if __name__ == '__main__':

    (env,options)=oh.setup('qtcoin',True)
    env.SetDebugLevel(3)
    #TODO fix hard name here
    options.robotfile='../robots/drchubo/drchubo_v3/robots/drchubo_v3.robot.xml'
    [robot,ctrl,ind,ref,recorder]=oh.load_scene(env,options)

    rightFootSpheres=ContactSphereSet()
    rightFootSpheres.append(ContactSphere(robot,Transform(trans=[0.063,0.058,-0.005]),'rightFoot'))
    rightFootSpheres.append(ContactSphere(robot,Transform(trans=[0.063,-0.058,-0.005]),'rightFoot'))
    rightFootSpheres.append(ContactSphere(robot,Transform(trans=[-0.093,0.058,-0.005]),'rightFoot'))
    rightFootSpheres.append(ContactSphere(robot,Transform(trans=[-0.093,-0.058,-0.005]),'rightFoot'))

    check = CWSCheck(robot)
    check.insert_contacts('rightFoot', rightFootSpheres)
    check.build_active_sets()

