from math import *
from decimal import *

# Read the file to obtain time steps and the total time
file_read = open("parameters")
getcontext().prec = 5

print cos(90)
print cos(pi/2)

#line 1 
stringer_width=0
line = file_read .readline()
counter=0;
length =len(line)
for parser in range(0, length):
	if line[parser]=='\t':
		counter=parser
if not line:
	print "stringer width Missing"
else:
	stringer_width=Decimal(line[0:counter-1])/2
	print "stringer_width   "+str(stringer_width)
#line 2 
line = file_read .readline()
counter=0;
length =len(line)
for parser in range(0, length):
	if line[parser]=='\t':
		counter=parser
stringer_height=0
if not line:
	print "stringer height Missing"
else:
	stringer_height=Decimal(line[0:counter-1])/2
	print "stringer_height   "+str(stringer_height)


#line 3 
line = file_read .readline()
counter=0;
length =len(line)
for parser in range(0, length):
	if line[parser]=='\t':
		counter=parser
stringer_thickness=0
if not line:
	print "stringer thickness Missing"
else:
	stringer_thickness=Decimal(line[0:counter-1])/2
	print "stringer_thickness   "+str(stringer_thickness)


#line 4 
line = file_read .readline()
counter=0;
length =len(line)
for parser in range(0, length):
	if line[parser]=='\t':
		counter=parser
rung_width=0
if not line:
	print "rung width Missing"
else:
	rung_width=Decimal(line[0:counter-1])/2
	print "rung_width   "+str(rung_width)


#line 5 
line = file_read .readline()
counter=0;
length =len(line)
for parser in range(0, length):
	if line[parser]=='\t':
		counter=parser
rung_height=0
if not line:
	print "rung height Missing"
else:
	rung_height=Decimal(line[0:counter-1])/2
	print "rung_height   "+str(rung_height)

#line 6 
line = file_read .readline()
counter=0;
length =len(line)
for parser in range(0, length):
	if line[parser]=='\t':
		counter=parser
rung_thickness=0
if not line:
	print "rung thickness Missing"
else:
	rung_thickness=Decimal(line[0:counter-1])/2

#line 7 
line = file_read .readline()
counter=0;
length =len(line)
for parser in range(0, length):
	if line[parser]=='\t':
		counter=parser
ladder_angle=0
if not line:
	print "ladder angle Missing"
else:
	ladder_angle=Decimal(line[0:counter-1])
ladder_angle_rad = ladder_angle*Decimal(pi/180)

#line 8 
line = file_read .readline()
counter=0;
length =len(line)
for parser in range(0, length):
	if line[parser]=='\t':
		counter=parser
rung_space=0
if not line:
	print "rung space Missing"
else:
	rung_space=Decimal(line[0:counter-1])


#line 9 
line = file_read .readline()
counter=0;
length =len(line)
for parser in range(0, length):
	if line[parser]=='\t':
		counter=parser
stringer_radius=0
if not line:
	print "stringer_radius Missing"
else:
	print line[0:counter-1]
	stringer_radius=Decimal(line[0:counter-1])
print 'stringer radius is  ' +str(stringer_radius)

#line 10 
line = file_read .readline()
counter=0;
length =len(line)
for parser in range(0, length):
	if line[parser]=='\t':
		counter=parser
stringer_nface=0
if not line:
	print "stringer_nface Missing"
else:
	stringer_nface=Decimal(line[0:counter-1])

#line 11 
line = file_read .readline()
counter=0;
length =len(line)
for parser in range(0, length):
	if line[parser]=='\t':
		counter=parser
rung_radius=0
if not line:
	print "rung_radius Missing"
else:
	rung_radius=Decimal(line[0:counter-1])


#line 12 
line = file_read .readline()
counter=0;
length =len(line)
for parser in range(0, length):
	if line[parser]=='\t':
		counter=parser
rung_nface=0
if not line:
	print "rung_nface Missing"
else:
	rung_nface=Decimal(line[0:counter-1])


#line 13 
line = file_read .readline()
counter=0;
length =len(line)
for parser in range(0, length):
	if line[parser]=='\t':
		counter=parser
rung_shape=0
if not line:
	print "rung shape Missing"
else:
	rung_shape=Decimal(line[0:counter-1])


#line 14
line = file_read .readline()
counter=0;
length =len(line)
for parser in range(0, length):
	if line[parser]=='\t':
		counter=parser
stringer_shape=0
if not line:
	print "stringer shape Missing"
else:
	stringer_shape=Decimal(line[0:counter-1])

no_of_rungs=0
if stringer_shape==1:
	no_of_rungs=2*stringer_height/rung_space
else:
	no_of_rungs=stringer_nface/rung_space
	
file_read .close()
print 'partsing done'

cos_ladder_angle=float(cos(ladder_angle))
sin_ladder_angle=float(sin(ladder_angle))
if stringer_shape==1:
	stringer_l=stringer_height
else:
	stringer_l=stringer_nface

file_write = open("ladder_env.xml", "wb")


file_write.write('<Environment>\n')
file_write.write('  <!-- set the background color of the environment-->\n')
file_write.write('  <bkgndcolor>0.0 0.0 0.</bkgndcolor>\n')
file_write.write(' \n')
file_write.write('  <!-- import the segway model and place it somewhere-->\n')
file_write.write('  <KinBody file="ladder.xml">\n')
file_write.write('  <!--<Translation>0 (Decimal(cos(ladder_angle))) 0 </Translation>-->\n')
file_write.write('    <Translation>0 '+str(2*Decimal(cos(ladder_angle_rad))*stringer_l)+' 0</Translation>\n')
file_write.write('  <!--<RotationAxis>1 0 0 theta</RotationAxis>-->\n')
file_write.write('    <RotationAxis>1 0 0 '+str(90-ladder_angle)+'</RotationAxis>\n')
file_write.write('  </KinBody>\n')
file_write.write('\n')

file_write.write('\n')
file_write.write('  <!-- add the floor as a box-->\n')
file_write.write('  <KinBody name="floor">\n')
file_write.write('    <!-- floor should never move, so make it static-->\n')
file_write.write('    <Body type="static">\n')
file_write.write('      <Geom type="box">\n')
file_write.write('        <extents>10 10 1</extents>\n')
file_write.write('        <diffuseColor>0.6 0.6 0.6</diffuseColor>\n')
file_write.write('        <ambientColor>0.6 0.6 0.6</ambientColor>\n')
file_write.write('	      <Translation>0 0 -1</Translation>\n')
file_write.write('      </Geom>\n')
file_write.write('    </Body>\n')
file_write.write('  </KinBody>\n')
file_write.write('\n')
file_write.write('  <!-- add the wall as a box-->\n')
file_write.write('  <KinBody name="wall">\n')
file_write.write('    <!-- floor should never move, so make it static-->\n')
file_write.write('    <Body type="static">\n')
file_write.write('      <Geom type="box">\n')
file_write.write('          <extents>10 1 5</extents>\n')
file_write.write('          <diffuseColor>.9 .1 .6</diffuseColor>\n')
file_write.write('      	<ambientColor>0.6 0.6 0.6</ambientColor>\n')
file_write.write('	        <Translation>0 -1 5</Translation>\n')
file_write.write('      </Geom>\n')
file_write.write('    </Body>\n')
file_write.write('  </KinBody>\n')
file_write.write('\n')
file_write.write(' <physicsengine type="ode">\n')
file_write.write('  <odeproperties>\n')
file_write.write('   <friction>.5</friction>\n')
file_write.write('   <gravity>0 0 -9.80</gravity>\n')
file_write.write('   <selfcollision>1</selfcollision>\n')
file_write.write('   <erp>.5</erp>\n')
file_write.write('   <cfm>.000001</cfm>\n')
file_write.write('   <dcontactapprox>1</dcontactapprox>\n')
file_write.write('  </odeproperties>\n')
file_write.write(' </physicsengine>\n')
file_write.write('\n')
file_write.write('\n')
file_write.write('</Environment>\n')
