function [xd, dxd, ddxd] = xd_rettilinea(x_start, x_goal, tf, t_step)
    
    % dati traiettoria della posizione
    pos0 = x_start(1:3);
    delta_pos = x_goal(1:3) - x_start(1:3);
    delta_pos_norm = norm(delta_pos);

    % dati traiettoria dell'orientamento
    phi0 = x_start(4:6);
    delta_phi = x_goal(4:6) - x_start(4:6);
    delta_phi_norm = norm(delta_phi);
    
    % inizializzazione
    t = 0.0;
    xd = zeros(6, round(tf/t_step)+1);
    dxd = zeros(6, round(tf/t_step)+1);
    ddxd = zeros(6, round(tf/t_step)+1);
    
    while (t <= tf)
        % posizione
        s = L * (10 * (t/tf)^3 - 15 * (t/tf)^4 + 6 * (t/tf)^5);
        xd(1:3, t/t_step + 1) = pos0 + delta_pos / delta_pos_norm * s;
        % velocità
        ds = L/tf * (30 * (t/tf)^2 - 60 * (t/tf)^3 + 30 * (t/tf)^4);
        dxd(1:3, t/t_step + 1) = delta_pos / delta_pos_norm * ds;

        % accellerazione
        dds = L/(tf^2) * (60 * (t/tf) - 180 * (t/tf)^2 + 120 * (t/tf)^3);
        ddxd(1:3, t/t_step + 1) = delta_pos / delta_pos_norm * dds;

        % orientamento
        s_o = delta_phi_norm / L * s;
        xd(4:6, t/t_step + 1) = phi0 + delta_phi / delta_phi_norm * s_o;
        
        % velocità angolare
        dso = delta_phi_norm / L * ds;
        dxd(4:6, t/t_step + 1) = delta_phi / delta_phi_norm * dso;
        
        % accellerazione angolare
        ddso = delta_phi_norm / L * dds;
        ddxd(4:6, t/t_step + 1) = delta_phi / delta_phi_norm * ddso;

        t = t + t_step;
    end
end