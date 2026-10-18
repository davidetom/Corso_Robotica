function [p, phi, R, quat] = direct_kin(q, euler_angles)

    % Definizione dei limiti fisici dei giunti del braccio sinistro
    q_min = [-pi, -pi/2, -4.7124, -3.8208, -0.35, 1.2208, -pi/2];
    q_max = [ pi,  pi/2,  pi/2,   -1.4708,  0.35, 1.9208,  pi/2];

    % Controllo di violazione dei limiti
    if any(q < q_min) || any(q > q_max)
        warning('Cinematica Diretta: Uno o più giunti in ingresso superano i limiti strutturali.');
        
        % Forzo i valori di q all'interno del range consentito
        q = max(q_min, min(q, q_max));
    end

    load('matrici_robot.mat', 'A_b_e'); % Carica A_b_e

    syms theta_1 theta_2 theta_3 theta_4 theta_5 theta_6 theta_7

    % 3. Sostituisci i valori e converti in matrice numerica (double)
    A = double(subs(A_b_e, ...
    [theta_1, theta_2, theta_3, theta_4, theta_5, theta_6, theta_7], ...
    q));

    % Vettore di posizione
    p = A(1:3, 4);

    % Matrice di rotazione
    R = A(1:3, 1:3);

    % Angoli di Eulero
    if strcmpi(euler_angles, 'ZYZ')
        s_theta = sqrt(R(1,3)^2 + R(2,3)^2);

        if s_theta > 1e-6
            gamma   = atan2(R(2,3), R(1,3));
            theta   = atan2(s_theta, R(3,3));
            psi     = atan2(R(3,2), -R(3,1));
        else
            gamma = 0;
            if R(3,3) > 0 % theta = 0
                theta = 0;
                psi = atan2(R(2,1), R(1,1));
            else          % theta = pi
                theta = pi;
                psi = atan2(-R(2,1), -R(1,1));
            end 
        end
        
        phi = [gamma; theta; psi];

    elseif strcmpi(euler_angles, 'RPY')
        c_theta = sqrt(R(1,1)^2 + R(2,1)^2);
    
        if c_theta > 1e-6
            gamma   = atan2(R(2,1), R(1,1));
            theta   = atan2(-R(3,1), c_theta);
            psi     = atan2(R(3,2), R(3,3));
        else
            gamma = 0;
            if -R(3,1) > 0 % theta = pi/2
                theta = pi/2;
                psi = atan2(R(1,2), R(2,2));
            else           % theta = -pi/2
                theta = -pi/2;
                psi = atan2(-R(1,2), R(2,2));
            end
        end

        phi = [gamma; theta; psi];
    end
    
    % Quaternion vector
    nu = 0.5 * sqrt(R(1,1) + R(2,2) + R(3,3) + 1);
    epsilon_1 = 0.5 * sign(R(3,2) - R(2,3)) * sqrt(R(1,1) - R(2,2) - R(3,3) + 1);
    epsilon_2 = 0.5 * sign(R(1,3) - R(3,1)) * sqrt(R(2,2) - R(1,1) - R(3,3) + 1);
    epsilon_3 = 0.5 * sign(R(2,1) - R(1,2)) * sqrt(R(3,3) - R(1,1) - R(2,2) + 1);

    quat = [nu; epsilon_1; epsilon_2; epsilon_3];

end