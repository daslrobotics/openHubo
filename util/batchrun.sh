#Massive added torque

T="10.0"
for robot in 'robots/rlhuboplus.robot.xml' 'robots/rlhuboplus.noshell.robot.xml'
do
    for override in -1
    do
        for param in parameters/[789]*.iuparam
        do
            FNAME=`basename $param`
            PREFIX=${FNAME:0:(-8)}
            echo $PREFIX

            DEST_FOLDER=batchexperiment-$PREFIX
            mkdir $DEST_FOLDER
            python iuvalidate.py "parameters/$PREFIX.iuparam" "trajectories/$PREFIX.iutraj" "$robot" "$T" "$override" "0"
            echo "Copying files to $DEST_FOLDER"
            mv *.pickle *.avi *.log $DEST_FOLDER
            #Allow batch to be halted by user with SIGINT
            echo "Waiting for user input, continuing to next experiment in:"
            echo "3 seconds..."
            sleep 1
            echo "2 seconds..."
            sleep 1
            echo "1 second..."
            sleep 1
            echo "Continuing..."
        done
    done
done
