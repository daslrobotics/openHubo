
%Refresh the envrironment
orEnvLoadScene('jaemiHubo.leftarm.kinbody.xml',1);armID=orEnvGetBody('LeftArm');

for k=0:pi/10:2*pi
    orBodySetJointValues(armID,[0 0 0 0 0 k]);
pause(.2);
end

