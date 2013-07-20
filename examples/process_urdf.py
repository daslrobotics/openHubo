from openhubo.urdf import URDF
import sys
from openhubo import startup
from numpy import pi
from openhubo import hubo_util

try:
    filename=sys.argv[1]
except IndexError:
    filename=None

model=URDF.load_xml_file(filename)
#Mirror arm and leg chains
model.copy_chain_with_rottrans('Body_TSY','Body_RAR',[0,0,0],[0,0,0],r'R\([HKASEWF][RPY123]\)',r'L\1',True)
model.copy_chain_with_rottrans('Body_Torso','Body_RWR',[0,0,0],[0,0,0],r'R\([HKASEWF][RPY123]\)',r'L\1',True)

model.move_chain_with_rottrans('Body_RWR','Body_RF3',[0,0,0],[0,0,0],r'F([0-9])',r'F1\1')
#Original finger renamed
model.copy_chain_with_rottrans('Body_RWR','Body_RF13',[0,0,0],[0,0.045,0],r'F1',r'F2')
model.copy_chain_with_rottrans('Body_RWR','Body_RF13',[0,0,pi],[-0.046,-0.0015,0],r'F1',r'F3')
model.copy_chain_with_rottrans('Body_RWR','Body_RF13',[0,0,pi],[-0.046,-0.04675,0],r'F1',r'F4')


model.move_chain_with_rottrans('Body_LWR','Body_LF3',[0,0,0],[0,0,0],r'F([0-9])',r'F1\1')
#Original finger renamed
model.copy_chain_with_rottrans('Body_LWR','Body_LF13',[0,0,0],[0,-0.045,0],r'F1',r'F2')
model.copy_chain_with_rottrans('Body_LWR','Body_LF13',[0,0,pi],[-0.046,0.0015,0],r'F1',r'F3')

#Read in actual model limits from file
adjuster=hubo_util.LimitProcessor('/etc/hubo-ach/drc-hubo.table','/etc/hubo-ach/drc-hubo-beta-2-default.home')
adjuster.apply_limits(model)


model.write_xml('test.urdf')
