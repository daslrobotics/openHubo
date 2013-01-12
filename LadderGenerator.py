from math import *
from decimal import *
import sys

## Ladder Format
#0.05		#ladder-stringer-width
#3			#ladder-stringer-height
#0.05		#ladder-stringer-thickness
#0.6			#ladder-rung-width
#0.05		#ladder-rung-height
#0.10		#ladder-rung-thickness
#65			#ladder-angle
#0.20		#ladder-rung-space
#0.025		#ladder-stringer-radius
#20			#ladder-stringer-nface
#0.03		#ladder-rung-radius
#20			#ladder-rung-nface
#0			#1-rect--0-cir-rung-cross-section
#1			#1-rect--0-cir-stringer-cross-section


# Read the file to obtain time steps and the total time
def make_ladder_env(paramfile):
    file_read = open(paramfile,'r')
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
    print 'Parsing done'

    cos_ladder_angle=float(cos(ladder_angle))
    sin_ladder_angle=float(sin(ladder_angle))
    if stringer_shape==1:
        stringer_l=stringer_height
    else:
        stringer_l=stringer_nface

    #Kludgy interpretation of posix path here, may choke on special chars
    paramname=paramfile.split('/')[-1]
    name_tokens=paramname.split('.')
    if len(name_tokens)>1:
        outname='.'.join(name_tokens[:-1])
    else:
        outname=name_tokens[-1]

    outname+='.env.xml'
    print outname
    file_write = open(outname, "wb")

    file_write.write('<Environment>\n')
    file_write.write('  <!-- set the background color of the environment-->\n')
    file_write.write('  <bkgndcolor>0.0 0.0 0.</bkgndcolor>\n')
    file_write.write(' \n')
    file_write.write('  <KinBody file="{}">\n'.format(outname[:-8]+'.kinbody.xml'))
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
    file_write.close()
    return outname

def make_ladder(paramfile):
    # Read the file to obtain time steps and the total time
    with open(paramfile) as file_read:
        getcontext().prec = 5

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
            
    print 'Parsing done'

    ## Writing the ladder.xml file
    paramname=paramfile.split('/')[-1]
    name_tokens=paramname.split('.')
    if len(name_tokens)>1:
        outname='.'.join(name_tokens[:-1])
    else:
        outname=name_tokens[-1]

    outname+='.kinbody.xml'
    file_write = open(outname, 'w')
    file_write.write('<KinBody name="ladder">\n')
    file_write.write('	    	<Body name="Base" type="static">\n')
    file_write.write('	      		<Translation>0.0  0.0  0.0</Translation>\n')
    file_write.write('\n')

    ## Writing the Stringers
    # Case cylinder
    if stringer_shape==0:
        file_write.write(' 			<!-- Left Vertical-->\n')
        file_write.write('   			<Geom type="cylinder">\n')
        file_write.write('				<radius>'+str(stringer_radius)+'</radius>\n')
        file_write.write('				<height>'+str(stringer_height)+'</height>\n')
        file_write.write('			        <!-- set the translation and rotation of the box. Note that the transforamtions-->\n')
        file_write.write('			        <!-- are all relative to the parent body.-->\n')
        if rung_shape==1:
            file_write.write('      			<Translation> ' + str(-rung_width-10*stringer_width/2)+' 0 '+str(stringer_nface/2) +'</Translation>\n')
        else:
            file_write.write('      			<Translation> ' + str(-rung_width-10*stringer_width/2)+' 0 '+str(stringer_nface/2) +'</Translation>\n')
        file_write.write('        			<RotationAxis>1 0 0 90</RotationAxis>\n')
        file_write.write('    			</Geom>\n')
        file_write.write('\n')

        file_write.write(' 			<!-- Right Vertical-->\n')
        file_write.write('   			<Geom type="cylinder">\n')
        file_write.write('				<radius>'+str(stringer_radius)+'</radius>\n')
        file_write.write('				<height>'+str(stringer_height)+'</height>\n')
        file_write.write('			        <!-- set the translation and rotation of the box. Note that the transforamtions-->\n')
        file_write.write('			        <!-- are all relative to the parent body.-->\n')
        if rung_shape==1:
            file_write.write('      			<Translation> ' + str(rung_width+10*stringer_width/2)+' 0 '+str(stringer_nface/2) +'</Translation>\n')
        else:
            file_write.write('      			<Translation> ' + str(rung_width+10*stringer_width/2)+' 0 '+str(stringer_nface/2) +'</Translation>\n')
        file_write.write('        			<RotationAxis>1 0 0 90</RotationAxis>\n')
        file_write.write('    			</Geom>\n')
        file_write.write('\n')

    ## Case cuboid
    else:
        file_write.write(' 			<!-- Left Vertical-->\n')
        file_write.write('			<Geom type="box"> \n')
        file_write.write('   				<!-- extents of the box - half width, height, length-->\n')
        file_write.write('      			<Extents>'+str(stringer_width)+' '+str(stringer_thickness)+' '+str(stringer_height)+' </Extents>\n')
        file_write.write('			        <!-- set the translation and rotation of the box. Note that the transforamtions-->\n')
        file_write.write('			        <!-- are all relative to the parent body.-->\n')
        if rung_shape==1:
            file_write.write('      			<Translation>' + str(-rung_width-10*stringer_width/2)+' 0 '+str(stringer_height)+' </Translation>\n')
        else:
            file_write.write('      			<Translation>' + str(-rung_width-10*stringer_width/2)+' 0 '+str(stringer_height)+' </Translation>\n')
        file_write.write('        			<RotationAxis>1 0 0 0</RotationAxis>\n')
        file_write.write('			</Geom>\n')
        file_write.write('\n')
        file_write.write(' 			<!-- Right Vertical-->\n')
        file_write.write('			<Geom type="box"> \n')
        file_write.write('   				<!-- extents of the box - half width, height, length-->\n')
        file_write.write('      			<Extents>'+str(stringer_width)+' '+str(stringer_thickness)+' '+str(stringer_height)+' </Extents>\n')
        file_write.write('			        <!-- set the translation and rotation of the box. Note that the transforamtions-->\n')
        file_write.write('			        <!-- are all relative to the parent body.-->\n')
        if rung_shape==1:
            file_write.write('      			<Translation>' + str(rung_width+10*stringer_width/2)+' 0 '+str(stringer_height)+' </Translation>\n')
        else:
            file_write.write('      			<Translation>' + str(rung_width+10*stringer_width/2)+' 0 '+str(stringer_height)+' </Translation>\n')
        file_write.write('        			<RotationAxis>1 0 0 0</RotationAxis>\n')
        file_write.write('			</Geom>\n')


    ## RUNGS
    ## case rect rungs
    if rung_shape==1:
        for i in range(1,no_of_rungs+1):
            file_write.write('\n')
            file_write.write(' 			<!-- '+str(i)+'th Step-->\n')
            file_write.write('   			<Geom type="box">\n')
            file_write.write('   				<!-- extents of the box - half width, height, length-->\n')
            file_write.write('      			<Extents>'+str(rung_width+10*stringer_width)+' '+str(rung_thickness)+ ' '+str(rung_height)+' </Extents>\n')
            file_write.write('			        <!-- set the translation and rotation of the box. Note that the transforamtions-->\n')
            file_write.write('			        <!-- are all relative to the parent body.-->\n')
            file_write.write('      			<Translation>0 0 '+str(i*rung_space)+'</Translation>\n')
            file_write.write('        			<RotationAxis>1 0 0 0</RotationAxis>\n')
            file_write.write('    			</Geom>\n')
            file_write.write('\n')

    ## case cylinder rungs
    else:
        for i in range(1,no_of_rungs+1):
            file_write.write('\n')
            file_write.write(' 			<!-- '+str(i)+'th Step-->\n')
            file_write.write('   			<Geom type="cylinder">\n')
            file_write.write('				<radius>'+str(rung_radius)+'</radius>\n')
            file_write.write('				<height>'+str(2*rung_width+10*stringer_width)+'</height>\n')
            file_write.write('			        <!-- set the translation and rotation of the box. Note that the transforamtions-->\n')
            file_write.write('			        <!-- are all relative to the parent body.-->\n')
            file_write.write('      				<Translation>0 0 '+str(i*rung_space)+'</Translation>\n')
            file_write.write('        			<RotationAxis> 0 0 1 90</RotationAxis>\n')
            file_write.write('    			</Geom>\n')
            file_write.write('\n')

    ## End file
    file_write.write('		</Body>\n')
    file_write.write('</KinBody>\n')
    file_write.close()
    return outname

if __name__=='__main__':
    try:
        filename = sys.argv[1]
    except IndexError:
        filename = 'firstladder.iuparam'

    make_ladder(filename)

