% Calcola e salva offline la matrice Jacobiana geometrica simbolica
clear
close all

% 1. Carico le matrici parametriche
load('matrici_robot.mat', 'A_b_0', 'A_b_1', 'A_b_2', ...
    'A_b_3', 'A_b_4', 'A_b_5', 'A_b_6', 'A_b_e');

syms theta_1 theta_2 theta_3 theta_4 theta_5 theta_6 theta_7 real
theta_param = [theta_1, theta_2, theta_3, theta_4, theta_5, theta_6, theta_7];

% 2. Raggruppo le matrici parametriche
A_param = {A_b_0, A_b_1, A_b_2, A_b_3, A_b_4, A_b_5, A_b_6, A_b_e};
k = length(A_param);

% STRUTTURA DEL JACOBIANO GEOMETRICO
% |                             |
% |   z_(i-1) x (P_e - o_(i-1)) |
% |                             |
% |           z_(i-1)           |

% Estraggo le posizioni delle origini delle terne di ciascun giunto
P = sym(zeros(3, k));
for i = 1:k
    P(:, i) = A_param{i}(1:3, 4);
end

% Estraggo i versori z delle terne di ciascun giunto
z = sym(zeros(3, k));
for i = 1:k
    z(:, i) = A_param{i}(1:3, 3);
end

% Costruisco lo Jacobiano geometrico simbolico 6x7
J_sym = sym(zeros(6, 7));
for i = 1:7
    % Prime 3 righe dello Jacobiano (velocita' lineare)
    r = P(:, end) - P(:, i);
    J_sym(1:3, i) = simplify(cross(z(:, i), r));
    
    % Ultime 3 righe dello Jacobiano (velocita' angolare)
    J_sym(4:6, i) = simplify(z(:, i));
end

% 3. Salvataggio dello Jacobiano simbolico in un file .mat
save('jacobiano_geometrico.mat', 'J_sym', 'theta_param');
disp('Salvataggio dello Jacobiano geometrico completato');