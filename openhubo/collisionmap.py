import numpy as n
import re
from numpy import array,pi,interp,arange
from xml.dom import minidom
#from pylab import *

def write_reformatted(data,name):
    outdata=re.sub(r'\t','    ',data)
    with open(name,'w') as f:
        f.write(outdata)

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

    def insert_pair(self,pair,j0=None,j1=None):
        if j0 is not None:
            pair.j0=j0
        if j1 is not None:
            pair.j1=j1
        self.pairs.append(pair)

    def to_xml(self):
        outstrings=['    <collisionmap>']
        for r in self.pairs:
            outstrings.append(r.to_xml())

        outstrings.append('    </collisionmap>')
        return '\n'.join(outstrings)

    def write(self,robot):
        try:
            filename=robot.GetURI()
            name=robot.GetName()
        except AttributeError:
            print "robot {} is not loaded, assuming file name...".format(robot)
            filename=robot
            name=robot.split('.')[0]

        line1='<robot type="collisionmaprobot" name="{}">\n'.format(name)
        line2='    <robot file="{}"/>\n'.format(filename)
        lineN='\n</robot>\n'
        with open('cmap.'+filename,'w') as f:
            f.write(line1)
            f.write(line2)
            f.write(self.to_xml())
            f.write(lineN)


class JointCollisionPair:
    """Pair of colliding joints defined by a lookup table (see OpenRAVE docs for details).
        Define and analyze a lookup table for a joint map from a set of bounding values.
    """

    def __init__(self,filename,res,degrees=False):

        (j0_data,j1_data,j0,j1)=self.read_data(filename,degrees)

        #Assume raw pairs, preprocess
        (j0_bins,j1_min,j1_max)=self.preprocess_data(j0_data,j1_data,.5*pi/180)

        self.create_from_bounds(j0_bins,j1_min,j1_max,res)

        self.j0=j0
        self.j1=j1

    def create_from_bounds(self,points0,min1,max1,res_approx):
        """ Define a profile for a pair of joints and use it to convert to a lookup table.
        points0: A list of key values for joint 0, typically evenly spaced over the range of the joint, in RADIANS
        min1: A corresponding list of joint values for joint 1 for every position in points0, corresponding to the minimum joint angle.
        max1: Same as min1 but maximum values.
        res: Resolution of lookup table in radians
        """
        if res_approx<=0:
            raise ValueError('Resolution cannot be <= 0.0!')
        j0_min=n.min(points0)
        j0_max=n.max(points0)

        j1_min=n.min(min1)
        j1_max=n.max(max1)
        #print j1_min,j1_max

        #Find span of each joint table
        dj0=j0_max-j0_min
        dj1=j1_max-j1_min

        #Calculate exact resolution to get even steps:
        steps0=n.ceil(dj0/res_approx)
        steps1=n.ceil(dj1/res_approx)

        j0_values=n.linspace(j0_min,j0_max,steps0)
        j1_values=n.linspace(j1_min,j1_max,steps1)
        #print j1_values
        #TODO: better storage method? n arrays are better than lists, but still expensive
        self.table=n.zeros((len(j0_values),len(j1_values)),dtype=n.int)

        minbound=interp(j0_values,points0,min1)
        maxbound=interp(j0_values,points0,max1)
        #plot(points0,min1)
        #plot(points0,max1)
        #plot(min1)
        #Kludgy
        for (j,j0_val) in enumerate(j0_values):
            for (k,j1_val) in enumerate(j1_values):
                #print j,k,j1_val,maxbound[j]
                if j1_val<maxbound[j] and j1_val>minbound[j]:
                    self.table[j,k]=1
        self.j0_values=j0_values
        self.j1_values=j1_values

    def to_xml(self,j0=None,j1=None,indent='    '):
        if j0 is None:
            j0=self.j0
        if j1 is None:
            j1=self.j1

        pairdata=(n.shape(self.table),n.min(self.j0_values),n.min(self.j1_values),n.max(self.j0_values),n.max(self.j1_values),j0,j1)
        outlist=['{0}<pair dims="{1[0]} {1[1]}" min="{2} {3}" max="{4} {5}" joints="{6} {7}">'.format(indent*2,*pairdata)]
        for (j,j0_val) in enumerate(self.j0_values):
            dataline=[indent*3]
            dataline.extend(['{:d}'.format(self.table[j,k]) for k in xrange(n.size(self.table,1))])
            outlist.append(' '.join(dataline))

        outlist.append('{}</pair>'.format(indent*2))
        return '\n'.join(outlist)

    @staticmethod
    def preprocess_data(j0_raw,j1_raw,res):
        """Bin recorded data for joint pair by the provided resolution, and throw out interior points"""

        #plot(j0_raw,j1_raw)
        #figure()
        #Assume approximate resolution ok
        if len(j0_raw) != len(j1_raw):
            raise IndexError('Input arrays must be the same size')
        j0_mn=n.min(j0_raw)
        j0_mx=n.max(j0_raw)

        #Find span of each joint table
        dj0=j0_mx-j0_mn

        #Calculate exact resolution to get even steps:
        res_0=dj0/n.floor(dj0/res)
        bins=n.arange(j0_mn,j0_mx,res_0)

        #Arc length
        ds=n.sqrt(pow(n.diff(j0_raw),2.)+pow(n.diff(j1_raw),2.))
        ind=n.hstack((0,[i+1 for (i,s) in enumerate(ds) if abs(s) > 0.0000001 ]))
        asum=n.hstack((0,n.cumsum(ds)))

        #sample for bins
        arclen=[asum[i] for i in ind]
        S=max(arclen)-min(arclen)
        res_s=S/n.floor(4*S/res)
        steps=arange(min(arclen),max(arclen),res_s)
        j0_proc=n.interp(steps,arclen,[j0_raw[i] for i in ind])
        j1_proc=n.interp(steps,arclen,[j1_raw[i] for i in ind])
        #plot(j0_proc,j1_proc)
        indices=n.digitize(j0_proc,bins)
        #print indices.shape
        #plot(indices,j1_proc,'ro')
        #build a dict because it's easy
        j1_min={}
        j1_max={}
        for (i,j) in zip(indices,j1_proc):
            if (not j1_min.has_key(i)) or j1_min[i]>j:
                j1_min[i]=j

            if (not j1_max.has_key(i)) or j1_max[i]<j:
                j1_max[i]=j

        j0_out=[]
        j1_min_out=[]
        j1_max_out=[]

        for i in n.unique(indices):
            j0_out.append(bins[min(i,len(bins)-1)])
            j1_min_out.append(j1_min[i])
            j1_max_out.append(j1_max[i])

        #plot(j0_out,j1_min_out)
        #plot(j0_out,j1_max_out)
        return (j0_out,j1_min_out,j1_max_out)

    @staticmethod
    def write_source_data(j0_data,j1_data,name0,name1,filename):
        with open(filename,'w') as f:
            f.write(name0 + ',' + name1)
            for data in zip(j0_data,j1_data):
                f.write(','.join(data))

    @staticmethod
    def read_data(filename,degrees):
        with open(filename,'r') as f:
            header=f.readline().rstrip().split(',')
            j0=header[0]
            j1=header[1]
            j0_data=[]
            j1_data=[]
            scale=pi/180 if degrees else 1.
            for line in f:
                data=line.rstrip().split(',')
                j0_data.append(float(data[0])*scale)
                j1_data.append(float(data[1])*scale)
        return (j0_data,j1_data,j0,j1)

