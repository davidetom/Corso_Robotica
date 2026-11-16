function [Ja] = analytical_jacobian(q, euler_angles)
% Calcola lo Jacobiano analitico (6x7) nella configurazione q
% per la rappresentazione degli angoli di Eulero specificata ('ZYZ' o 'RPY')

    % Calcolo online lo Jacobiano geometrico J e la matrice A_b_e
    [J, A] = geometric_jacobian(q);

    R = A(1:3, 1:3);

    if strcmpi(euler_angles, 'ZYZ')
        s_theta = sqrt(R(1,3)^2 + R(2,3)^2);

        if s_theta > 1e-6
            gamma = atan2(R(2,3), R(1,3));
            theta = atan2(s_theta, R(3,3));
            psi   = atan2(R(3,2), -R(3,1));
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

        T = [0, -sin(gamma), cos(gamma)*sin(theta);
             0,  cos(gamma), sin(gamma)*sin(theta);
             1,       0,          cos(theta)      ];

    elseif strcmpi(euler_angles, 'RPY')
        c_theta = sqrt(R(1,1)^2 + R(2,1)^2);

        if c_theta > 1e-6
            gamma = atan2(R(2,1), R(1,1));
            theta = atan2(-R(3,1), c_theta);
            psi   = atan2(R(3,2), R(3,3));
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

        T = [0, -sin(gamma), cos(gamma)*cos(theta);
             0,  cos(gamma), sin(gamma)*cos(theta);
             1,       0,         -sin(theta)      ];
    end

    Ja = [J(1:3, :);        % la sottomatrice di posizione rimane uguale
          T \ J(4:6, :)];   % la sottomatrice di orientamento viene pre-moltiplicata per T^-1
end
