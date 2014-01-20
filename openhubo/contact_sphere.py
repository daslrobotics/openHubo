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

