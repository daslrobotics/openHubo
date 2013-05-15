from openhubo.collisionmap import *

if __name__=='__main__':

    #Create an empty collisionmap
    example=CollisionMap()

    sourcefile='parameters/hip-pitch-roll.cmap.txt'
    print "Loading information from {}".format(sourcefile)

    example.insert_pair(JointCollisionPair(sourcefile,2.*pi/180,True))
    #Override names to create left side (this is ok since the values are symmetrical in this case)
    example.insert_pair(JointCollisionPair(sourcefile,2.*pi/180,True),'LHP','LHR')

    destfile='sample-collisionmap.xml'
    print "Write completed collisionmap to {}".format(destfile)
    example.write(destfile)

    print example.to_xml()

