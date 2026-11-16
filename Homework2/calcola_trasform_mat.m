function [A_array] = calcola_trasform_mat(q)
% Calcola online le matrici di trasformazione omogenea dalla base
% ai singoli giunti (A_b_0, A_b_1, ..., A_b_6, A_b_e) nella configurazione q
% e restituisce l'array di celle A_param = {A_b_0, ..., A_b_e}.

    % Normalizzo q come vettore riga 1x7 (supporta sia riga che colonna)
    q = q(:).';

    % tabella matriciale con parametri Denavit-Hartenberg
    Tab = [0,       0,      q(1),   -pi/2;
           0,       0,      q(2),   -pi/2;
           0,      -0.28,   q(3),   -pi/2;
           0.28,    0,      q(4),    pi/2;   % x4 verso il polso (prima: -0.28, -pi/2)
           0,       0,      q(5),   -pi/2;   % (prima: pi/2)
           0,       0,      q(6),    pi/2;
           0,      -0.11,   q(7),     pi;];

    % matrice di trasformazione da base a 0
    A_b_0 = [-0.0449,   0.9848,     0.1677,     0.0155;
             -0.2549,   -0.1736,    0.9513,     0.2000;
             0.9659,    0,          0.2588,     0.9960;
             0,         0,          0,          1;];

    % matrici di trasformazione da base a i
    A_mat = cell(1, 7);

    % matrice di trasformazione da base a i-esimo giunto
    A_b_i = A_b_0;

    for i = 1:7
        A = zeros(4, 4);
        A(4, 4) = 1;

        A(1, 1) = cos(Tab(i, 3));
        A(1, 2) = -sin(Tab(i, 3)) * cos(Tab(i, 4));
        A(1, 3) = sin(Tab(i, 3)) * sin(Tab(i, 4));
        A(1, 4) = Tab(i, 1) * cos(Tab(i, 3));

        A(2, 1) = sin(Tab(i, 3));
        A(2, 2) = cos(Tab(i, 3)) * cos(Tab(i, 4));
        A(2, 3) = -cos(Tab(i, 3)) * sin(Tab(i, 4));
        A(2, 4) = Tab(i, 1) * sin(Tab(i, 3));

        A(3, 1) = 0;
        A(3, 2) = sin(Tab(i, 4));
        A(3, 3) = cos(Tab(i, 4));
        A(3, 4) = Tab(i, 2);

        A_b_i = A_b_i * A;

        A_mat{i} = A_b_i; % matrice da base a i
    end

    % array completo delle matrici {A_b_0, A_b_1, ..., A_b_6, A_b_e}
    A_array = [{A_b_0}, A_mat];

end
