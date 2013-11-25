from openravepy import CollisionReport
from numpy import linalg,zeros, cross, array, pi
import openhubo as oh
from scipy.spatial import ConvexHull
import numpy as np

#TODO declare a vector of peak forces for each contact body?
def perform_cws(robot):
    env=robot.GetEnv()
    report=CollisionReport()
    env.CheckCollision(robot,report)
    print report.contacts
    CWS=[array([0,0,0,0,0])]

    for c in report.contacts:
        for theta in [0,pi/2,pi,3*pi/2]:
            w = zeros(6)
            #TODO add force limits here based on body
            w[0:3] = (c.norm+array([np.cos(theta),np.sin(theta),0])*.1) * 1000.
            w[3:] = cross(c.pos,w[0:3])
            CWS.append(w[0:5])

    w_g = zeros(6)

    r = oh.find_com(robot)
    m = oh.find_mass(robot)

    g = m*array([0,0,-9.81])

    w_g[0:3] = -g
    w_g[3:6] = cross(r,-g)

    #TODO evaluate convex hull of CWS here
    hull = ConvexHull(array(CWS))

    check_interior(hull,w_g[0:5])

    #TODO check if w_g is in interior?
    #TODO reduce CWS with simplified contacts
    #TODO stellate CWS
    return (hull,CWS, report)

def check_interior(hull,point):

    inside = True
    points=hull.points
    for n,s in enumerate(hull.equations):

        res = s[0:5].dot(point[0:5])+s[5]
        if res>0:
            inside = False
            print "point is outside of face {}".format(n)
        else:
            print "point inside of face P{}".format(n)

    return inside

if __name__ == '__main__':

    (env,options)=oh.setup('None',True)
    env.SetDebugLevel(3)
    options.robotfile='robots/drchubo/drchubo_v3/robots/drchubo_v3.robot.xml'
    [robot,ctrl,ind,ref,recorder]=oh.load_scene(env,options)
    env.StartSimulation(oh.TIMESTEP)
    print "Move robot to colliding position"
    T=robot.GetTransform()
    T[2,3]-=.0015
    robot.SetTransform(T)

    hull, CWS, report = perform_cws(robot)


