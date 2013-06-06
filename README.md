Major Contributions
===================

Robert Ellenberg: Author and primary developer of python and OpenRAVE plugins, CAD modeling and export processes

R.J. Gross: created the entire Hubo2 CAD model in Autodesk Inventor

Richard Vallett: Significant OpenRAVE work, updating and working with Hubo+ CAD and modeling, and formalizing Inventor URDF export

Dan Lofaro: Testing using hubo-ach, physics simulation, and extensive usage testing

Kiwon Sohn: Usage and testing with MATLAB and python for vehicle ingress / egress

Youngbum Jun: Testing and usage with pump / hose insertion, text trajectory definitions

KAIST Hubo Lab and Rainbow: Creating the excellent Hubo humanoid series, and teaching many DASL members about humanoid robotics.

Quick Setup Instructions
========================

To set up openHubo 0.7+, clone the repository:

    git clone git://github.com/daslrobotics/openHubo.git
    cd openHubo
    source ./setup

If you are upgrading from an older version, fetch and checkout release 0.7.1:

    git fetch
    git checkout -t release/0.7.1
    ./setup

Choose 'y' at the first prompt to clean up old plugins and submodules and force
a rebuild. This will reduce the likelihood of issues down the road.

Dependencies
============

Make sure that you have all necessary dependencies installed to run both
openrave and this project. OpenHubo 0.7.x requires
fd1cee1130a7a14f33f2f7c25d42ab1cf14fa261 (25 March, 2013) or later to run
properly. The latest master branch of openrave is recommended.

While the build script knows to install a few common
dependencies, it is not gauranteed that all of them will be included. For
example, to view reachability databases with openrave, you will need the mayavi
package:

    sudo apt-get install mayavi2

OpenHubo can now be launched from a common executable:

    openhubo

Several examples are provided in the examples folder to get you started with
python and MATLAB programming with openHubo. To run examples, use the --example switch:

    openhubo --example servo_control.py 

Tab completion shows other available examples.

If you would like to run the achread example, make sure that hubo-ach is
installed and configurated, then rebuild the servocontroller plugin with ACH support enabled.

https://github.com/hubo/hubo-ach

If you have any issues with the models or OpenHubo, please submit an issue to
the github issue tracker. Happy Simulating!

Using Older Versions
==================

To use older versions of OpenHubo, simple checkout the appropriate branch or
tag, and run the setup script as above. Note that 0.8.x represents significant
structural changes and deprecates some old functions. Due to the new openhubo
python package, this may break existing code. The release/0.7.0 branch is
provided for compatibility, if your code breaks using 0.8+.

