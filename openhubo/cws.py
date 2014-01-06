from openravepy import CollisionReport
from numpy import linalg,zeros, cross, array, pi
import openhubo as oh
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
    points=hull.points
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

if __name__ == '__main__':

    (env,options)=oh.setup('qtcoin',True)
    env.SetDebugLevel(3)
    #TODO fix hard name here
    options.robotfile='../robots/drchubo/drchubo_v3/robots/drchubo_v3.robot.xml'
    [robot,ctrl,ind,ref,recorder]=oh.load_scene(env,options)
    env.StartSimulation(oh.TIMESTEP)
    print "Move robot to colliding position"
    T=robot.GetTransform()
    T[2,3]-=.0015
    robot.SetTransform(T)
    oh.pause()

    stable, hull, CWS, report = perform_cws(robot,['leftFoot','rightFoot','leftPalm','rightPalm'])


