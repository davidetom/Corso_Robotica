function [xd, dxd, ddxd] = xd_arco_circ(q, x_start, x_goal, tf, t_step)
    
    A = calcola_trasform_mat(q);
    A_gomito = A{end-4};

    % dati traiettoria della posizione (centro al gomito)
    pos_centro = A_gomito(1:3,4);
    u = x_start(1:3) - pos_centro;
    raggio = norm(u);
    v = x_goal(1:3) - pos_centro;

    % costruzione della terna locale del cerchio R_loc = [x_loc, y_loc, z_loc]
    x_loc = u / raggio;
    z_cross = cross(u, v);
    z_loc = z_cross / norm(z_cross);
    y_loc = cross(z_loc, x_loc);
    R_loc = [x_loc, y_loc, z_loc];
    theta = atan2(norm(z_cross), dot(u, v));
    L = raggio * theta;

    % dati traiettoria dell'orientamento
    phi_start = x_start(4:6);
    phi_goal = x_goal(4:6);
    % normalizzazione di delta_phi in [-pi, pi] per evitare salti
    delta_phi = atan2(sin(phi_goal - phi_start), cos(phi_goal - phi_start));
    
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

        % posizione 
        s = L * sigma;
        xd(1:3, k) = pos_centro + R_loc * ...
            [raggio * cos(s / raggio); 
             raggio * sin(s / raggio); 
             0];

        % velocità
        ds = L * dsigma;
        dxd(1:3, k) = R_loc * ...
            [-sin(s / raggio) * ds; 
             cos(s / raggio) * ds; 
             0];

        % accelerazione
        dds = L * ddsigma;
        ddxd(1:3, k) = R_loc * ...
            [-cos(s / raggio) * ds^2 / raggio - sin(s / raggio) * dds;
             -sin(s / raggio) * ds^2 / raggio + cos(s / raggio) * dds;
             0];

        % orientamento e derivate degli angoli
        xd(4:6, k) = phi_start + delta_phi * sigma;
        dxd(4:6, k) = delta_phi * dsigma;
        ddxd(4:6, k) = delta_phi * ddsigma;
    end
end