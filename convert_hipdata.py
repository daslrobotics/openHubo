from openhubo.collisionmap import *
import fnmatch
import os,sys
from parse_hubo_read import *

if __name__=='__main__':
    #fname=''
    #for f in os.listdir('.'):
        #if fnmatch.fnmatch(f, sys.argv[1]):
            #print f
            #fname = f
            #break

    #import cProfile, openhubo.startup
    #pr = cProfile.Profile()
    #pr.enable()
    #hist=hubo_history(fname)
    #pr.disable()
    #pr.print_stats('time')

    #hist.write_joint_pair('LHP','LHR','LHP-LHR.pair')
    #hist.write_joint_pair('RSP','RSR','RSP-RSR.pair')
    #hist.write_joint_pair('RAP','RAR','RAP-RAR.pair')
    #hist.write_joint_pair('LAP','LAR','LAP-LAR.pair')
    #Create an empty collisionmap
    example=CollisionMap()

    sourcefile='LHP-LHR.pair'
    #sourcefile='RAP-RAR.pair'
    #sourcefile='RSP-RSR.pair'
    print "Loading information from {}".format(sourcefile)

    example.insert_pair(JointCollisionPair(sourcefile,1.*pi/180,False))
    #Override names to create left side (this is ok since the values are symmetrical in this case)
    #example.insert_pair(JointCollisionPair(sourcefile,2.*pi/180,True),'LHP','LHR')

    destfile='example-collisionmap.xml'
    print "Write completed collisionmap to {}".format(destfile)
    example.write(destfile)

    print example.to_xml()

