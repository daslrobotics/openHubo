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
