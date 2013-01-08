from numpy import pi,array
import openravepy as rave

jointmap={"RHY":26, "RHR":27, "RHP":28, "RKN":29, "RAP":30, "RAR":31, "LHY":19, "LHR":20, "LHP":21, "LKN":22, "LAP":23, "LAR":24, "RSP":11, "RSR":12, "RSY":13, "REB":14, "RWY":15, "RWR":16, "RWP":17, "LSP":4, "LSR":5, "LSY":6, "LEB":7, "LWY":8, "LWR":9, "LWP":10, "NKY":1, "NK1":2, "NK2":3, "WST":0, "RF1":32, "RF2":33, "RF3":34, "RF4":35, "RF5":36, "LF1":37, "LF2":38, "LF3":39, "LF4":40, "LF5":41}

def get_achname_from_name(inname):
    name=inname.encode('ASCII')
    #Cheat a little since names are roman characters
    if (name == "LKP" or name == "RKP"): 
        achname=name[0:2]+'N'
    
    elif (name == "LEP" or name == "REP"): 
        achname=name[0:2]+'B'
    
    elif (name == "HPY" or name == "TSY"): 
        achname = "WST"
    
    elif (name == "HNY"): 
        achname="NKY"
    
    elif (name == "HNR"): 
        achname="NK1"
    
    elif (name == "HNP"): 
        achname="NK2"
    
    elif (name == "leftIndexKnuckle1"): 
        achname="LF2"
    
    elif (name == "leftMiddleKnuckle1"): 
        achname="LF3"
    
    elif (name == "leftRingKnuckle1"): 
        achname="LF4"
    
    elif (name == "leftPinkyKnuckle1"): 
        achname="LF5"
    
    elif (name == "leftThumbKnuckle1"): 
        achname="LF1"
    
    elif (name == "rightIndexKnuckle1"): 
        achname="RF2"
    
    elif (name == "rightMiddleKnuckle1"): 
        achname="RF3"
    
    elif (name == "rightRingKnuckle1"): 
        achname="RF4"
    
    elif (name == "rightPinkyKnuckle1"): 
        achname="RF5"
    
    elif (name == "rightThumbKnuckle1"): 
        achname="RF1"
    else:
        achname=name

    if jointmap.has_key(achname):
        return achname
    else:
        return None

def build_joint_index_map(robot):
    jointlist=zeros(robot.GetDOF())-1
    for j in robot.GetJoints():
        name=j.GetName()
        print name
        achname=get_achname_from_name(name)
        print achname
        if achname:
            jointlist[j.GetDOFIndex()]=jointmap[achname]
    return jointlist
           
if __name__=='__main__':
    from openravepy import *
    from servo import *
    env=Environment()
    #env.SetViewer('qtcoin')
    [robot,ctrl,ind,refrobot]=openhubo.load_rlhuboplus(env)
    env.StartSimulation(timestep=0.0005)

    outputmap=build_joint_index_map(robot)
    print outputmap
