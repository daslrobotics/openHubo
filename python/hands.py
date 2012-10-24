from openravepy import *

#TODO: Work with the concept of activeDOF?
def makeNameToIndexConverter(robot):
    def convert(name):
        return robot.GetJoint(name).GetDOFIndex()
    return convert

def setupMimicJoints(robot,mimicjoints,sourcejoints=None,scale=1,offset=0):
    if sourcejoints==None:
        for k in range(len(mimicjoints)):
            setupMimicJoint(robot,mimicjoints[k])
    else:
        for k in range(len(mimicjoints)):
            setupMimicJoint(robot,mimicjoints[k],sourcejoints[k],scale,offset)



def setupMimicJoint(robot,mimic,source=None,scale=1,offset=0):

    joint=robot.GetJoint(mimic)

    #assume hinge or slider joints for now
    if source==None:
        joint.SetMimicEquations(0,"")
    else:
        joint1=robot.GetJoint(source)
        print "{}*{}+{}".format(joint1.GetName(),scale,offset)
        joint.SetMimicEquations(0,"{}*{}+{}".format(joint1.GetName(),scale,offset),"|{} {}".format(joint1.GetName(),scale),"")

def setupFingerMimic(robot):
    knuckles=[]
    source=[]
    for s in ['left','right']:
        for f in ['Index','Middle','Ring','Pinky','Thumb']:
            for k in [2,3]:
                knuckles.append('{}{}Knuckle{}'.format(s,f,k))
                source.append('{}{}Knuckle1'.format(s,f))

    print knuckles
    print source

    setupMimicJoints(robot,knuckles,source,1,0)

