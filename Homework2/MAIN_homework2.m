% Main script for Homework2
clear all 
close all
addpath(genpath('/Applications/coppeliaSim.app/Contents/Resources/programming/zmqRemoteApi/clients/matlab'));

client = RemoteAPIClient();
sim = client.getObject('sim');

% Joint space configuration
qDH = [0,-pi/3,-pi/2,-pi, 0, 0.6981, 0];

% Joint space limits
qMinDH = [-3.1416   -1.5708   -4.7124   -3.8208   -1.3963    0.1745   -1.5708]'; %rad x7
qMaxDH = [3.1416    1.5708    1.5708   -1.4708    1.3963    2.9671    1.5708]'; %rad x7

simulator = SimProjectUtils(sim); 
simulator.moveLeftArm(qDH);

%% Useful functions to grasp/release objects: 
% simulator.graspObject();
% simulator.releaseObject();
