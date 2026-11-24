function [J, A_b_e] = geometric_jacobian(q)
% Calcola lo Jacobiano geometrico (6x7) nella configurazione q
% utilizzando le matrici di trasformazione restituite da calcola_trasform_mat(q).

    % 1. Ottengo le matrici di trasformazione {A_b_0, A_b_1, ..., A_b_6, A_b_e}
    A_array = calcola_trasform_mat(q);
    k = length(A_array);
    A_b_e = A_array{end};

    % STRUTTURA DELLO JACOBIANO GEOMETRICO
    % |                             |
    % |   z_(i-1) x (P_e - o_(i-1)) |
    % |                             |
    % |           z_(i-1)           |
    % |                             |

    % Estraggo le posizioni delle origini delle terne di ciascun giunto
    P = zeros(3, k);
    for i = 1:k
        P(:, i) = A_array{i}(1:3, 4);
    end

    % Estraggo i versori z delle terne di ciascun giunto
    z = zeros(3, k);
    for i = 1:k
        z(:, i) = A_array{i}(1:3, 3);
    end

    % Costruisco lo Jacobiano geometrico numerico 6x7
    J = zeros(6, 7);
    for i = 1:7
        % Prime 3 righe dello Jacobiano (velocita' lineare)
        r = P(:, end) - P(:, i);
        J(1:3, i) = cross(z(:, i), r);

        % Ultime 3 righe dello Jacobiano (velocita' angolare)
        J(4:6, i) = z(:, i);
    end

end
