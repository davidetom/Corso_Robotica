clear
close all

% incognite ai giunti
syms theta_1 theta_2 theta_3 theta_4 theta_5 theta_6 theta_7 real

theta = [theta_1, theta_2, theta_3, theta_4, theta_5, theta_6, theta_7];

% tabella matriciale con parametri Denavit-Hartenberg
Tab = [0,       0,      theta_1,   -pi/2;
       0,       0,      theta_2,   -pi/2;
       0,      -0.28,   theta_3,   -pi/2;
       0.28,    0,      theta_4,    pi/2;   % x4 verso il polso (prima: -0.28, -pi/2)
       0,       0,      theta_5,   -pi/2;   % (prima: pi/2)
       0,       0,      theta_6,    pi/2;
       0,      -0.11,   theta_7,     pi;];

% matrice di trasformazione da base a 0
A_b_0 = [-0.0449,   0.9848,     0.1677,     0.0155;
         -0.2549,   -0.1736,    0.9513,     0.2000;
         0.9659,    0,          0.2588,     0.9960;
         0,         0,          0,          1;];

% matrici di trasformazione da base a i
A_mat = cell(1,7);

% matrice di trasformazione da base a end effector
A_b_i = sym(A_b_0);

for i = 1:7
    A = sym(zeros(4,4));
    A(4,4) = 1;

    A(1,1) = cos(Tab(i,3));
    A(1,2) = -sin(Tab(i,3))*cos(Tab(i,4));
    A(1,3) = sin(Tab(i,3))*sin(Tab(i,4));
    A(1,4) = Tab(i,1)*cos(Tab(i,3));

    A(2,1) = sin(Tab(i,3));
    A(2,2) = cos(Tab(i,3))*cos(Tab(i,4));
    A(2,3) = -cos(Tab(i,3))*sin(Tab(i,4));
    A(2,4) = Tab(i,1)*sin(Tab(i,3));
    
    A(3,1) = 0;
    A(3,2) = sin(Tab(i,4));
    A(3,3) = cos(Tab(i,4));
    A(3,4) = Tab(i,2);

    A_b_i = simplify(A_b_i * A); 

    A_mat{i} = simplify(A_b_i); % matrice da base a i
end

% assegnazione matrici di trasformazione da 0 a e
A_b_1 = A_mat{1};
A_b_2 = A_mat{2};
A_b_3 = A_mat{3};
A_b_4 = A_mat{4};
A_b_5 = A_mat{5};
A_b_6 = A_mat{6};
A_b_e = A_mat{7};

% salvataggio matrici in un file .mat richiamabile dal main
% salvo tutte le matrici per completezza anche se basterebbe A_b_e
save('matrici_robot.mat', 'A_b_0', 'A_b_1', 'A_b_2', 'A_b_3', 'A_b_4', 'A_b_5', 'A_b_6', 'A_b_e');
disp('Salvataggio completato');
