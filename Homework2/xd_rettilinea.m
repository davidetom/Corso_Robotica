function [xd, dxd, ddxd] = xd_rettilinea(x_start, x_goal, tf, t_step)
    
    % dati traiettoria della posizione
    pos0 = x_start(1:3);
    delta_pos = x_goal(1:3) - x_start(1:3);
    L = norm(delta_pos);

    % dati traiettoria dell'orientamento
    phi0 = x_start(4:6);
    % normalizzazione di delta_phi in [-pi, pi] per evitare salti
    delta_phi = atan2(sin(x_goal(4:6) - x_start(4:6)), cos(x_goal(4:6) - x_start(4:6)));
    
    % inizializzazione
    N = round(tf / t_step) + 1;
    xd = zeros(6, N);
    dxd = zeros(6, N);
    ddxd = zeros(6, N);
    
    for k = 1:N
        t = (k - 1) * t_step;
        tau = t / tf;

        % legge oraria polinomiale di 5° grado normalizzata in [0, 1]
        sigma   = 10 * tau^3 - 15 * tau^4 + 6 * tau^5;
        dsigma  = (30 * tau^2 - 60 * tau^3 + 30 * tau^4) / tf;
        ddsigma = (60 * tau - 180 * tau^2 + 120 * tau^3) / (tf^2);

        % posizione, velocità, accelerazione lineare
        xd(1:3, k) = pos0 + delta_pos * sigma;
        dxd(1:3, k) = delta_pos * dsigma;
        ddxd(1:3, k) = delta_pos * ddsigma;

        % orientamento e relative derivate degli angoli di Eulero
        xd(4:6, k) = phi0 + delta_phi * sigma;
        dxd(4:6, k) = delta_phi * dsigma;
        ddxd(4:6, k) = delta_phi * ddsigma;
    end
end