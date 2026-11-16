clear all
close all 

% 1. Aggiungi il percorso della libreria ZMQ di CoppeliaSim
%addpath(genpath('/Applications/coppeliaSim.app/Contents/Resources/programming/zmqRemoteApi/clients/matlab'));

% 2. Inizializza il client ZMQ e ottieni l'oggetto 'sim'
%client = RemoteAPIClient();
%sim = client.getObject('sim');

% 3. Passa l'oggetto 'sim' alla classe SimProjectUtils
%simulator = SimProjectUtils(sim); 

% 4.a.
% posa iniziale
q0 = [0, -pi/2, -pi/2, pi/4, 0, -pi/2, 0]; 
[p0, phi0, ~, ~] = direct_kin(q0, 'ZYZ');                                             
xd = [p0 + [0; 0; 0.05]; phi0];
dxd = 0;
ddxd = 0;
Tmax = 10;

[X,dX,ddX,Q,dQ,ddQ] = inverse_kin(q0, xd, dxd, ddxd, Tmax);

% Plot dell'evoluzione nello spazio operativo e nello spazio dei giunti
t = (0:size(X, 1)-1) * (Tmax / size(X, 1));

figure('Name', 'Spazio Operativo');
subplot(3, 2, 1);
plot(t, X(:, 1:3), 'LineWidth', 1.2); grid on;
title('Posizione p'); ylabel('[m]'); legend('x', 'y', 'z');

subplot(3, 2, 2);
plot(t, X(:, 4:6), 'LineWidth', 1.2); grid on;
title('Orientamento \phi (ZYZ)'); ylabel('[rad]'); legend('\gamma', '\theta', '\psi');

subplot(3, 2, 3);
plot(t, dX(:, 1:3), 'LineWidth', 1.2); grid on;
title('Velocità lineare dp'); ylabel('[m/s]'); legend('dx', 'dy', 'dz');

subplot(3, 2, 4);
plot(t, dX(:, 4:6), 'LineWidth', 1.2); grid on;
title('Velocità angoli di Eulero d\phi'); ylabel('[rad/s]'); legend('d\gamma', 'd\theta', 'd\psi');

subplot(3, 2, 5);
plot(t, ddX(:, 1:3), 'LineWidth', 1.2); grid on;
title('Accelerazione lineare ddp'); xlabel('t [s]'); ylabel('[m/s^2]'); legend('ddx', 'ddy', 'ddz');

subplot(3, 2, 6);
plot(t, ddX(:, 4:6), 'LineWidth', 1.2); grid on;
title('Accelerazione angoli di Eulero dd\phi'); xlabel('t [s]'); ylabel('[rad/s^2]'); legend('dd\gamma', 'dd\theta', 'dd\psi');

figure('Name', 'Spazio dei Giunti');
subplot(3, 1, 1);
plot(t, Q, 'LineWidth', 1.2); grid on;
title('Posizione giunti Q'); ylabel('[rad]');
legend('q_1', 'q_2', 'q_3', 'q_4', 'q_5', 'q_6', 'q_7', 'Location', 'bestoutside');

subplot(3, 1, 2);
plot(t, dQ, 'LineWidth', 1.2); grid on;
title('Velocità giunti dQ'); ylabel('[rad/s]');
legend('dq_1', 'dq_2', 'dq_3', 'dq_4', 'dq_5', 'dq_6', 'dq_7', 'Location', 'bestoutside');

subplot(3, 1, 3);
plot(t, ddQ, 'LineWidth', 1.2); grid on;
title('Accelerazione giunti ddQ'); xlabel('t [s]'); ylabel('[rad/s^2]');
legend('ddq_1', 'ddq_2', 'ddq_3', 'ddq_4', 'ddq_5', 'ddq_6', 'ddq_7', 'Location', 'bestoutside');



% 4.b Muovi il braccio
%simulator.moveLeftArm(q);
%pause(1);

% 5. Test della posizione dell'end-effector su 20 configurazioni
%    (in alternativa: test_pose(sim, simulator, Q, dt) con Q matrice N x 7)
%err = test_pose(sim, simulator);