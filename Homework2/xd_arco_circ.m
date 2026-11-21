function [xd, dxd, ddxd] = xd_arco_circ(q, x_start, x_goal, tf, t_step)
    
    A = calcola_trasform_mat(q);
    A_gomito = A{end-4};
    mat_trans = A_gomito(1:3, 1:3);
    
    % dati traiettoria della posizione
    pos_centro = A_gomito(1:3,4);
    u = x_start(1:3) - pos_centro;
    raggio = norm(u);
    v = x_goal(1:3) - pos_centro;
    theta = atan2(norm(cross(u,v)), dot(u,v));
    L = raggio * theta;

    % dati traiettoria dell'orientamento
    phi_start = x_start(4:6);
    phi_goal = x_goal(4:6);
    delta_phi = phi_goal - phi_start;
    delta_phi_norm = norm(delta_phi);
    
    % inizializzazione
    t = 0.0;
    xd = zeros(6, round(tf/t_step)+1);
    dxd = zeros(6, round(tf/t_step)+1);
    ddxd = zeros(6, round(tf/t_step)+1);
    
    while (t <= tf)
        % posizione
        s = L * (10 * (t/tf)^3 - 15 * (t/tf)^4 + 6 * (t/tf)^5);
        xd(1:3, t/t_step + 1) = centro + mat_trans' * ...
            [raggio * cos(s / raggio); 
             raggio * sin(s / raggio); 
             0];

        % velocità
        ds = L/tf * (30 * (t/tf)^2 - 60 * (t/tf)^3 + 30 * (t/tf)^4);
        dxd(1:3, t/t_step + 1) = mat_trans' * ...
            [-sin(s / raggio) * ds; 
             cos(s / raggio) * ds; 
             0];

        % accellerazione
        dds = L/(tf^2) * (60 * (t/tf) - 180 * (t/tf)^2 + 120 * (t/tf)^3);
        ddxd(1:3, t/t_step + 1) = mat_trans' * ...
            [-cos(s / raggio) * ds^2 / raggio - sin(s / raggio) * dds;
             -sin(s / raggio) * ds^2 / raggio + cos(s / raggio) * dds;
             0];

        % orientamento
        s_o = delta_phi_norm / L * s;
        xd(4:6, t/t_step + 1) = phi_start + delta_phi / delta_phi_norm * s_o;
        
        % velocità angolare
        dso = delta_phi_norm / L * ds;
        dxd(4:6, t/t_step + 1) = delta_phi / delta_phi_norm * dso;
        
        % accellerazione angolare
        ddso = delta_phi_norm / L * dds;
        ddxd(4:6, t/t_step + 1) = delta_phi / delta_phi_norm * ddso;

        t = t + t_step;
    end
end