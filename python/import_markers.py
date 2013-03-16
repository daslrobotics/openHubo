#!/usr/bin/env python
import sys
from openravepy import *
from numpy import array

def read_marker_file(filename):
    #Setup trajectory and source file
    f=open(filename,'r')

    markersets={}
    while True: 
        string=f.readline().rstrip()
        if len(string)==0:
            break

        datalist=string.split(',')
        if len(datalist)==1:
            linkname=datalist[0]
            markersets.setdefault(linkname,[])
        else:
            point=[float(x) for x in datalist]
            markersets[linkname].append(point)
    
    return markersets

def write_xml(markersets,forceoffset=None,filename='markers.xml',scale=1.):
    

    if forceoffset is None:
        forceoffset={}
        for k in markersets.keys():
            forceoffset.setdefault(k,[0,0,0])
    with open(filename,'w') as f:
        for k in markersets.keys():
            f.write('<!--Body {} -->\n'.format(k))
            
            for point in markersets[k]:
                f.write('<geom type="sphere">\n')
                f.write('    <!-- Inverse COM Offset -->\n')
                f.write('    <translation>{} {} {}</translation>\n'.format(*forceoffset[k]))
                f.write('    <!-- Local marker Location -->\n')
                location=array(point)*scale
                f.write('    <translation>{} {} {}</translation>\n'.format(*location))
                f.write('    <radius>.008</radius>\n') 
                f.write('</geom>\n')


def plot_markers(markerset,robot):
    pass
    #env=robot.GetEnv()

    #for k in markerset.keys():
        #T=robot.GetTransform(robot.GetLink(k))
        
        #handle=env.plot3(points=array(markers),pointsize=8,colors=array([.8,.8,.8]),drawstyle=1)
    #return handle



if __name__=='__main__':

    filename = sys.argv[1]

    m=read_marker_file(filename)
    offsets={}
    offsets.setdefault('Body_Torso',[0.012258116293,0.002473149928,0.048635606065])
    #Get from model
    offsets.setdefault('Body_RAR',[0,0,0])
    write_xml(m,offsets,'markers.xml',0.001)
