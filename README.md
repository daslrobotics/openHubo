Authors: Robert Ellenberg, Richard Vallett, and R.J. Gross
To setup plugins for openrave, source setup-plugins.sh:

    source ./setup-plugins.sh

If you have any trouble building, or have conflicts with your current setup,
it may be easier to clean up everything completely, and rebuild. From the
openHubo folder:

    ./cleanup-all.sh
    git submodule update --init

At this point, the build folders in the submodules will be removed completely,
forcing CMake to rerun for each plugin. This is useful if you upgrade your
openrave installation and want to be sure the plugins are built against the
latest headers.

Also make sure that you have all necessary dependencies installed to run both
openrave and this project. While the build script knows to install a few common
dependencies, it is not gauranteed that all of them will be included. For
example, to view reachability databases with openrave, you will need the mayavi
package:

    sudo apt-get install mayavi2

Several examples are provided in the examples folder to get you started with
python and MATLAB programming with openHubo. 

If you would like to run the achcontroller example, make sure that hubo-ach is
installed (see the github repo for instructions)

https://github.com/hubo/hubo-ach

Assuming you have downloaded the repository in ~/hubo-ach, you can use the
included script to start hubo-ach:

./ach-run.sh

This creates the ACH channels for the hubo's reference pose, state, and
parameters. The script also starts the hubo main loop, which sends CAN messages
over virtual or physical CAN interfaces. The script is configured by default to
send only virtual CAN messages, since the necessary CAN hardware is not likely
installed on your PC. TO use phsyical can channels, remove the -v flag in the
last line of the script:

sudo ~/hubo-ach/hubo-main -v
