for T in 3.0 4.0 5.0 6.0 7.0 8.0 9.0 10.0 11.0 12.0 13.0 
do
    DEST_FOLDER=batchexperiment-$T
    mkdir $DEST_FOLDER
    echo "Copying files to $DEST_FOLDER"
    python iuvalidate.py path_70_0.20_5cm.iuparam trajectories/path_70_0.20.iutraj "$T"
    python iuvalidate.py path_70_0.20_6cm.iuparam trajectories/path_70_0.20.iutraj "$T"
    python iuvalidate.py path_70_0.20_2x4.iuparam trajectories/path_70_0.20.iutraj "$T"
    python iuvalidate.py path_70_0.20_3x5.iuparam trajectories/path_70_0.20.iutraj "$T"
    python iuvalidate.py path_70_0.20_4x6.iuparam trajectories/path_70_0.20.iutraj "$T"
    #python iuvalidate.py path_70_0.20_thick.iuparam trajectories/path_70_0.20.iutraj "$T"
    #python iuvalidate.py path_80_0.25.iuparam trajectories/path_80_0.25.iutraj "$T"
    #python iuvalidate.py path_90_0.25.iuparam trajectories/path_90_0.25.iutraj "$T"

    mv *.pickle *.avi $DEST_FOLDER
    
    sleep 1
    
done
