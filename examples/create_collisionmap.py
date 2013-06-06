from openhubo import collisionmap as cmap
import openhubo
from numpy import pi

if __name__=='__main__':

    #Create an empty collisionmap
    example=cmap.CollisionMap()

    sourcefile='parameters/hip-pitch-roll.cmap.txt'
    print "Loading information from {}".format(sourcefile)

    example.insert_pair(cmap.JointCollisionPair(sourcefile,2.*pi/180,True))
    #Override names to create left side (this is ok since the values are symmetrical in this case)
    example.insert_pair(cmap.JointCollisionPair(sourcefile,2.*pi/180,True),'LHP','LHR')

    robotfile='rlhuboplus.robot.xml'
    print "Write completed collisionmap robot to {}".format(robotfile)

    example.write(robotfile)

    print example.to_xml()


