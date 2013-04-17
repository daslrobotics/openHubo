import numpy as n
import pylab as p
from numpy import array,pi,interp
import matplotlib.pyplot as plt
import IPython
from mpl_toolkits.mplot3d import Axes3D

class CollisionMap:
    """Collision Map class for OpenRAVE CollisionMapRobot.
        This is primarily a wrapper for collision pairs to facilitate XML export."""

    def __init__(self,pair=None,*args):
        #No protection against double insert
        self.pairs=[]
        if type(pair) is type(list):
            for r in pair:
                self.insert_pair(r)
        elif type(pair) is type(JointCollisionPair):
            self.insert_pair(pair)
        elif pair is not None:
            self.insert_pair(JointCollisionPair(pair,*args))

    def insert_pair(self,pair):
        self.pairs.append(pair)

    def to_xml(self):
        outstrings=['<collisionmap>']
        for r in self.pairs:
            outstrings.append(r.to_xml())

        outstrings.append('</collisionmap>')
        return '\n'.join(outstrings)

    def write(self,filename=None,robot=None):
        if filename is None:
            if robot is not None:
                filename=robot.GetName()+'.collisionmap.xml'
            else:
                raise ValueError('No filename or robot provided!')

        with open(filename,'w') as f:
            f.write(self.to_xml())



class JointCollisionPair:
    """Pair of colliding joints defined by a lookup table (see OpenRAVE docs for details).
        Define and analyze a lookup table for a joint map from a set of bounding values.
    """

    def __init__(self,points0,min1,max1,res=None,j0=None,j1=None):
        if res is None:
            res=.5*pi/180
        self.create_from_bounds(points0,min1,max1,res)
        self.j0=j0
        self.j1=j1

    def create_from_bounds(self,points0,min1,max1,res):
        """ Define a profile for a pair of joints and use it to convert to a lookup table.
        points0: A list of key values for joint 0, typically evenly spaced over the range of the joint, in RADIANS
        min1: A corresponding list of joint values for joint 1 for every position in points0, corresponding to the minimum joint angle.
        max1: Same as min1 but maximum values.
        res: Resolution of lookup table in radians
        """
        if res<=0:
            raise ValueError('Resolution cannot be <= 0.0!')
        j0_min=n.min(points0)
        j0_max=n.max(points0)

        j1_min=n.min(min1)
        j1_max=n.max(max1)

        j0_values=n.arange(j0_min,j0_max,res)
        j1_values=n.arange(j1_min,j1_max,res)
        #TODO: better storage method? n arrays are better than lists, but still expensive
        self.table=n.zeros((len(j0_values),len(j1_values)),dtype=n.int)

        minbound=interp(j0_values,points0,min1)
        maxbound=interp(j0_values,points0,max1)
        #Kludgy
        for (j,j0_val) in enumerate(j0_values):
            for (k,j1_val) in enumerate(j1_values):
                if j1_val<maxbound[j] and j1_val>minbound[j]:
                    self.table[j,k]=1
        self.j0_values=j0_values
        self.j1_values=j1_values

    #def create_from_points(self,points,res):
        #sort(points,

    def plot(self):
        fig = plt.figure()
        ax = Axes3D(fig)
        Y,X=n.meshgrid(self.j1_values,self.j0_values)
        ax.plot_surface(X,Y, self.table)

    def to_xml(self,j0=None,j1=None):
        if j0 is None:
            j0=self.j0
        if j1 is None:
            j1=self.j1

        pairdata=(n.shape(self.table),n.min(self.j0_values),n.min(self.j1_values),n.max(self.j0_values),n.max(self.j1_values),j0,j1)
        outlist=['    <pair dims="{0[0]} {0[1]}" min="{1} {2}" max="{3} {4}" joints="{5} {6}">'.format(*pairdata)]
        for (j,j0_val) in enumerate(self.j0_values):
            dataline=['{:d}'.format(self.table[j,k]) for k in xrange(n.size(self.table,1))]
            #Kludge tab insert
            dataline.insert(0,'     ')
            outlist.append(' '.join(dataline))

        outlist.append('    </pair>')
        return '\n'.join(outlist)


def _angle_to_point(point, centre):
    '''calculate angle in 2-D between points and x axis'''
    delta = point - centre
    res = n.arctan(delta[1] / delta[0])
    if delta[0] < 0:
        res += n.pi
    return res


def _draw_triangle(p1, p2, p3, **kwargs):
    tmp = n.vstack((p1,p2,p3))
    x,y = [x[0] for x in zip(tmp.transpose())]
    p.fill(x,y, **kwargs)


def area_of_triangle(p1, p2, p3):
    '''calculate area of any triangle given co-ordinates of the corners'''
    return n.linalg.norm(n.cross((p2 - p1), (p3 - p1)))/2.


def convex_hull(points, graphic=True, smidgen=0.0075):
    '''Calculate subset of points that make a convex hull around points

Recursively eliminates points that lie inside two neighbouring points until only convex hull is remaining.

:Parameters:
    points : ndarray (2 x m)
        array of points for which to find hull
    graphic : bool
        use pylab to show progress?
    smidgen : float
        offset for graphic number labels - useful values depend on your data range

:Returns:
    hull_points : ndarray (2 x n)
        convex hull surrounding points
'''
    if graphic:
        p.clf()
        p.plot(points[0], points[1], 'ro')
    n_pts = points.shape[1]
    assert(n_pts > 5)
    centre = points.mean(1)
    if graphic: p.plot((centre[0],),(centre[1],),'bo')
    angles = n.apply_along_axis(_angle_to_point, 0, points, centre)
    pts_ord = points[:,angles.argsort()]
    if graphic:
        for i in xrange(n_pts):
            p.text(pts_ord[0,i] + smidgen, pts_ord[1,i] + smidgen, \
                   '%d' % i)
    pts = [x[0] for x in zip(pts_ord.transpose())]
    prev_pts = len(pts) + 1
    k = 0
    while prev_pts > n_pts:
        prev_pts = n_pts
        n_pts = len(pts)
        if graphic: p.gca().patches = []
        i = -2
        while i < (n_pts - 2):
            Aij = area_of_triangle(centre, pts[i],     pts[(i + 1) % n_pts])
            Ajk = area_of_triangle(centre, pts[(i + 1) % n_pts], \
                                   pts[(i + 2) % n_pts])
            Aik = area_of_triangle(centre, pts[i],     pts[(i + 2) % n_pts])
            if graphic:
                _draw_triangle(centre, pts[i], pts[(i + 1) % n_pts], \
                               facecolor='blue', alpha = 0.2)
                _draw_triangle(centre, pts[(i + 1) % n_pts], \
                               pts[(i + 2) % n_pts], \
                               facecolor='green', alpha = 0.2)
                _draw_triangle(centre, pts[i], pts[(i + 2) % n_pts], \
                               facecolor='red', alpha = 0.2)
            if Aij + Ajk < Aik:
                if graphic: p.plot((pts[i + 1][0],),(pts[i + 1][1],),'go')
                del pts[i+1]
            i += 1
            n_pts = len(pts)
        k += 1
    return n.asarray(pts)



if __name__=='__main__':
    HipPitch=array([0,40,50,60,70,80,85,90,92,100,110])*pi/180
    MinRoll=array([-32,-32,-30,-26.5,-23.4,-6,-6,-16,-4,-4,-16])*pi/180
    MaxRoll=array([32,32,30,26.5,23.4,6,6,16,4,4,16])*pi/180

    #Hack to get mirror about 0
    RHP=n.hstack((n.flipud(-HipPitch[1:]),HipPitch))
    RHR_min=n.hstack((n.flipud(MinRoll[1:]),MinRoll))
    RHR_max=n.hstack((n.flipud(MaxRoll[1:]),MaxRoll))

    rlhuboplus=CollisionMap()
    rlhuboplus.insert_pair(JointCollisionPair(RHP,RHR_min,RHR_max,1.*pi/180,'RHP','RHR'))
    rlhuboplus.insert_pair(JointCollisionPair(RHP,RHR_min,RHR_max,1.*pi/180,'LHP','LHR'))

    rlhuboplus.write('collisionmap.xml')
    #IPython.embed()

