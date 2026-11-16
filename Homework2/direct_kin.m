function [p, phi, R, quat] = direct_kin(q, euler_angles)

    A_array = calcola_trasform_mat(q);
    A = A_array{end};

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
    
    % Quaternione
    nu = 0.5 * sqrt(R(1,1) + R(2,2) + R(3,3) + 1);
    epsilon_1 = 0.5 * sign(R(3,2) - R(2,3)) * sqrt(R(1,1) - R(2,2) - R(3,3) + 1);
    epsilon_2 = 0.5 * sign(R(1,3) - R(3,1)) * sqrt(R(2,2) - R(1,1) - R(3,3) + 1);
    epsilon_3 = 0.5 * sign(R(2,1) - R(1,2)) * sqrt(R(3,3) - R(1,1) - R(2,2) + 1);

    quat = [nu; epsilon_1; epsilon_2; epsilon_3];

end