%% Example script to show loading a trajectory through matlab using Youngbum's format
%Make sure openrave is loaded

orEnvSetOptions('debug 4')
orEnvSetOptions('simulation stop')
orEnvLoadScene('scenes/simpleFloor.env.xml',1);
robotid = orEnvGetBody('hubo');

%set printing and display options

orRobotControllerSet(robotid,'servocontroller')

%TODO: troubleshoot errors here
