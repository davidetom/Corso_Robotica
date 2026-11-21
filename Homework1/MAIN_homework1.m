clear all
close all 

% 1. Aggiungi il percorso della libreria ZMQ di CoppeliaSim
addpath(genpath('/Applications/coppeliaSim.app/Contents/Resources/programming/zmqRemoteApi/clients/matlab'));

% 2. Inizializza il client ZMQ e ottieni l'oggetto 'sim'
client = RemoteAPIClient();
sim = client.getObject('sim');

% 3. Passa l'oggetto 'sim' alla classe SimProjectUtils
simulator = SimProjectUtils(sim); 

pause(3);
% 4.a. Calcola la cinematica diretta dell'end effector  
% posa di riferimento con la tabella DH con x4 verso il polso
q = [0, -pi/2, -pi/2, pi/4, 0, -pi/2, 0]; 
simulator.moveLeftArm(q);
pause(1);

% Configurazione a L
q = [-pi/3, -pi/2, -pi/2, pi/20, 0, -pi/2, 0];
simulator.moveLeftArm(q);
pause(1);

% Saluto - dx
q = [-pi/3, -pi/2, -pi/1.5, pi/20, 0, -pi/2, 0];
simulator.moveLeftArm(q);
pause(1);

% Saluto - sx
q = [-pi/3, -pi/2, -pi/3, pi/20, 0, -pi/2, 0];
simulator.moveLeftArm(q);
pause(1);

% Saluto - dx
q = [-pi/3, -pi/2, -pi/1.5, pi/20, 0, -pi/2, 0];
simulator.moveLeftArm(q);
pause(1);

% Saluto - sx
q = [-pi/3, -pi/2, -pi/3, pi/20, 0, -pi/2, 0];
simulator.moveLeftArm(q);
pause(1);

% Configurazione a L
q = [-pi/3, -pi/2, -pi/2, pi/20, 0, -pi/2, 0];
simulator.moveLeftArm(q);
pause(1);

%% 
%[p, phi, R, quat] = direct_kin(q, 'ZYZ'); % 'ZYZ' o 'RPY'
%disp(p);
%disp(phi);

% 4.b Muovi il braccio
simulator.moveLeftArm(q);
pause(1);

% 5. Test della posizione dell'end-effector su 20 configurazioni
%    (in alternativa: test_pose(sim, simulator, Q, dt) con Q matrice N x 7)
%err = test_pose(sim, simulator);