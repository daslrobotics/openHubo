import xml.etree.ElementTree as ET
from numpy import pi,array
import re
import itertools
#import debug
from copy import deepcopy
import sys

jointmask='L([SEWHKAF][PRY0-9])'
linkmask='Body_L([SEWHKAF][PRY0-9])'

def mirror_inertia(node):
    """ Mirror an inertia matrix about the Y axis"""
    inertia=node.find('inertia')
    if inertia is not None:
        for i in ['ixy','iyz']:
            inertia.set(i,'{:0.16e}'.format(-float(inertia.get(i))))

def mirror_origin(node):
    """Mirror an origin tag about the XZ plane"""
    origin=node.find('origin')
    if origin is None:
        return
    xyz=origin.get('xyz')
    xyzlist=[float(s) for s in xyz.split(' ')]
    xyzlist[1]=-xyzlist[1]
    xyz_new='{:0.17G} {:0.17G} {:0.17G}'.format(*xyzlist)
    origin.set('xyz',xyz_new)

    rpy=origin.get('rpy')
    rpylist=[float(s) for s in rpy.split(' ')]
    rpylist[0:3:2]=-array(rpylist[0:3:2])
    rpy_new='{:0.17G} {:0.17G} {:0.17G}'.format(*rpylist)
    origin.set('rpy',rpy_new)

def fix_mesh_case(node):
    """ For a given link, look through all child nodes for geometry tags, fixing extension case"""
    for c in node.getchildren():
        #print c
        for g in c.findall('geometry'):
            #print g
            mesh=g.find('mesh')
            #print mesh.get('filename')
            mesh.set('filename',re.sub(".STL",".stl",mesh.get('filename')))
            #print mesh.get('filename')

def mirror_geometry(node):
    """Update any geometry tags to refer to mirrored geometry, based on naming convention"""
    for g in node.findall('geometry'):
        mesh=g.find('mesh')
        mesh.set('filename',get_right_from_left(mesh.get('filename')))

def mirror_limits(node):
    """Update any geometry tags to refer to mirrored geometry, based on naming convention"""
    for g in node.findall('limit'):
        lower=float(g.get('lower'))
        upper=float(g.get('upper'))
        g.set('lower',str(-upper))
        g.set('upper',str(-lower))


def get_right_from_left(name):
    #joint
    name2=re.sub(jointmask,r"R\1",name,1)
    #link
    name3=re.sub(linkmask,r"Body_R\1",name2,1)
    return re.sub('left','right',name3,1)

def on_left_side(name):
    if re.search(linkmask,name) or re.search('left',name):
        return True
    if re.search(jointmask,name):
        return True
    return False


def make_mirror_node(node):
    """Make a new node if the name matches a left-to-right mirror.
        This depends entirely on all of the names being formatted correctly."""
    name=node.get('name')
    if not on_left_side(name):
        return None
    print name
    if node.tag == 'joint':
        #Copy link into mirrored list
        m=deepcopy(node)
        m.set('name',get_right_from_left(name))
        #Replace link names
        parent=m.find('parent')
        parent.set('link',get_right_from_left(parent.get('link')))
        child=m.find('child')
        child.set('link',get_right_from_left(child.get('link')))
        print node.find('parent').get('link')
    if node.tag == 'link':
        #Copy link into mirrored list
        m=deepcopy(node)
        m.set('name',get_right_from_left(name))
        #Replace mesh name if it exists

    return m

def run(sourcename,destname):

    tree = ET.parse(sourcename)
    links=tree.findall('link')

    mirlinks=[]
    for l in links:
        fix_mesh_case(l)
        m=make_mirror_node(l)
        if m is None:
            #Not a symmetrical body, ignore
            continue

        inertials=m.findall('inertial')
        visuals=m.findall('visual')
        collisions=m.findall('collision')
        nodes=[inertials,visuals,collisions]

        #Flip Y axis
        for i in list(itertools.chain(*nodes)):
            mirror_origin(i)
            mirror_inertia(i)
            mirror_geometry(i)
        mirlinks.append(m)

    mirjoints=[]

    for j in tree.findall('joint'):
        m=make_mirror_node(j)
        if m is None:
            #Not a symmetrical body, ignore
            continue
        mirror_origin(m)
        mirjoints.append(m)
        if re.search('[RL].[RY]',j.get('name')):
            #Match only roll and yaw joints
            mirror_limits(m)

    #Add new links and joints
    root=tree.getroot()
    root.extend(mirlinks)
    root.extend(mirjoints)

    tree.write(destname)

if __name__=='__main__':
    import startup
    """ Import an exported URDF file and create bilateral symmetry about the XZ plane"""
    try:
        filename=sys.argv[1]
    except IndexError:
        print "Please provide a file name to convert!"
        sys.exit(0)
    try:
        outname=sys.argv[2]
    except IndexError:
        print "Please provide an output filename"
        sys.exit(0)

    run(filename,outname)
