function [xd, dxd, ddxd] = trajectory()

    q0 = [0, -pi/3, -pi/2, 0, 0, 0.6981 - pi, 0];
    
    % Posa iniziale in spazio operativo
    [p0, phi0, ~, ~] = direct_kin(q0, 'ZYZ');
    x0 = [p0 ; phi0];

    % configurazione ai giunti del braccio pre-saluto
    q_goal_pre = [-pi/3, -pi/2, -pi/2, pi/20, 0, -pi/2, 0];
    

    % METODO PERCORSO ASSEGNATO AD ARCO DI CIRCONFERENZA
    
    % posizione dell'end effector "fine corsa dx" per il saluto
    q_goal_dx = [-pi/3, -pi/2, -2*pi/3, pi/20, 0, -pi/2, 0];
    [p_dx , phi_dx, ~, ~] = direct_kin(q_goal_dx, 'ZYZ');
    x_goal_dx = [p_dx ; phi_dx];

    % posizione dell'end effector "fine corsa sx" per il saluto
    q_goal_sx = [-pi/3, -pi/2, -pi/3, pi/20, 0, -pi/2, 0];
    [p_sx , phi_sx, ~, ~] = direct_kin(q_goal_sx, 'ZYZ');
    x_goal_sx = [p_sx ; phi_sx];
    
    % dati x traiettoria ad arco
    tf = 1.5;
    t_step = 0.001;
    [p, phi, ~, ~] = direct_kin(q_goal_pre, 'ZYZ');
    x_pre = [p ; phi];
    

    % METODO PERCORSO ASSEGNATO RETTILINEO
    
    % posizione della punta del cappello e orientamento ZYZ desiderato
    % variando solo gamma da pi/2 a 0 il cappello ruota solo attorno a zb restando dritto)
    phi_cap = [pi/2; 2*pi/3; -2*pi/3];
    x_goal_cap = [0.025; 0.75; 0.76; phi_cap];

    % posizione della testa del pupazzo di neve e orientamento ZYZ desiderato
    phi_head = [0; 2*pi/3; -2*pi/3];
    x_goal_head = [0.65; 0.13; 0.99; phi_head];

    % -------------------------------------------------------------------

    % COSTRUZIONE DI XD, DXD, DDXD COMPLESSIVE

    % 1° passo: pos iniziale -> pos pre-saluto
    [xd1, dxd1, ddxd1] = xd_rettilinea(x0, x_pre, 2*tf, t_step);

    % 2° passo: pos pre-saluto -> fine-corsa dx
    [xd2, dxd2, ddxd2] = xd_arco_circ(q_goal_pre, x_pre, x_goal_dx, tf, t_step);

    % 3° passo: fine-corsa dx -> fine-corsa sx
    [xd3, dxd3, ddxd3] = xd_arco_circ(q_goal_pre, x_goal_dx, x_goal_sx, 2*tf, t_step);

    % 4° passo: fine-corsa sx -> fine-corsa dx
    xd4 = xd3(:, end:-1:1);
    dxd4 = -dxd3(:, end:-1:1);
    ddxd4 = ddxd3(:, end:-1:1);

    % 5° passo: fine-corsa dx -> fine-corsa sx
    xd5 = xd3;
    dxd5 = dxd3;
    ddxd5 = ddxd3;

    % 6° passo: fine-corsa sx -> pos pre-saluto
    [xd6, dxd6, ddxd6] = xd_arco_circ(q_goal_pre, x_goal_sx, x_pre, tf, t_step);

    % 7° passo: pos pre-saluto -> pos cappello
    [xd7, dxd7, ddxd7] = xd_rettilinea(x_pre, x_goal_cap, 2*tf, t_step);

    % 8° passo: pos cappello -> pos testa pupazzo
    [xd8, dxd8, ddxd8] = xd_rettilinea(x_goal_cap, x_goal_head, 2*tf, t_step);

    % 9° passo: pos testa pupazzo -> pos iniziale
    [xd9, dxd9, ddxd9] = xd_rettilinea(x_goal_head, x0, 2*tf, t_step);
    

    % XD COMPLESSIVA (scartando il primo campione per evitare x duplicate)
    xd = [xd1, xd2(:, 2:end), xd3(:, 2:end), xd4(:, 2:end), xd5(:, 2:end), ...
          xd6(:, 2:end), xd7(:, 2:end), xd8(:, 2:end), xd9(:, 2:end)];

    % DXD COMPLESSIVA
    dxd = [dxd1, dxd2(:, 2:end), dxd3(:, 2:end), dxd4(:, 2:end), dxd5(:, 2:end), ...
           dxd6(:, 2:end), dxd7(:, 2:end), dxd8(:, 2:end), dxd9(:, 2:end)];

    % DDXD COMPLESSIVA
    ddxd = [ddxd1, ddxd2(:, 2:end), ddxd3(:, 2:end), ddxd4(:, 2:end), ddxd5(:, 2:end), ...
            ddxd6(:, 2:end), ddxd7(:, 2:end), ddxd8(:, 2:end), ddxd9(:, 2:end)];

    % [AGGIUNTA] Rende continui gli angoli di Eulero lungo il tempo evitando salti di 2*pi
    xd(4:6, :) = unwrap(xd(4:6, :), [], 2);

end