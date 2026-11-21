% Main script for Homework2
clear all 
close all
addpath(genpath('/Applications/coppeliaSim.app/Contents/Resources/programming/zmqRemoteApi/clients/matlab'));

client = RemoteAPIClient();
sim = client.getObject('sim');

% Joint space configuration
% Aggiornata qDH per coerenza con la nuova convenzione DH (x4 verso il polso),
% con i limiti qMinDH/qMaxDH e con trajectory.m.
% Vecchia convenzione (Homework2.pdf): qDH = [0,-pi/3,-pi/2,-pi, 0, 0.6981, 0];
qDH = [0, -pi/3, -pi/2, 0, 0, 0.6981 - pi, 0];

% Joint space limits (con x4 verso il basso)
qMinDH = [-3.1416   -1.5708   -4.7124   -0.6792   -1.3963    -2.9671   -1.5708]'; %rad x7
qMaxDH = [3.1416    1.5708    1.5708   1.6708    1.3963    -0.1745    1.5708]'; %rad x7

simulator = SimProjectUtils(sim);

[Xd, dXd, ddXd] = trajectory();
[x,dx,ddx,q,dq,ddq] = inverse_kin(qDH, Xd, dXd, ddXd);

N = size(q, 2);
for k = 1:N
    simulator.moveLeftArm(q(:, k));
    pause(0.001);
end

% simulator.moveLeftArm(qDH);

%% Useful functions to grasp/release objects: 
% simulator.graspObject();
% simulator.releaseObject();
