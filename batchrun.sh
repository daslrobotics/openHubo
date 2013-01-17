for T in 1.6 4.0 0.0 1.0 2.0 3.0 5.0 6.0
do
    DEST_FOLDER=batchexperiment-$T
    mkdir $DEST_FOLDER
    echo "Copying files to $DEST_FOLDER"
    python iuvalidate.py path_70_0.20.iuparam trajectories/path_70_0.20.iutraj "$T"
    python iuvalidate.py path_70_0.20_thick.iuparam trajectories/path_70_0.20.iutraj "$T"
    python iuvalidate.py path_80_0.25.iuparam trajectories/path_80_0.25.iutraj "$T"
    python iuvalidate.py path_90_0.25.iuparam trajectories/path_90_0.25.iutraj "$T"
    echo "\nExperimental batch completed, giving resulting files:\n" >> batch.log
    ls *.pickle >> batch.log

    mv *.pickle *.avi $DEST_FOLDER
    
    sleep 1
    
done
