function [xd, dxd, ddxd] = trajectory()

    q0 = [0,-pi/3,-pi/2,-pi, 0, 0.6981, 0];
    
    % METODO POLINOMIALE DEL 5° GRADO

    % configurazione ai giunti del braccio pre-saluto
    q_goal_pre = [-pi/3, -pi/2, -pi/2, pi/20, 0, -pi/2, 0];
    
    % configurazione ai giunti del braccio finale
    q_goal_fin = q0;
    

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
    [p, phi, ~, ~] = direct_kin(q, 'ZYZ');
    x_pre = [p ; phi];
    

    % METODO PERCORSO ASSEGNATO RETTILINEO
    
    % posizione della punta del cappello
    x_goal_cap = [0.025 0.75 0.76]';

    % posizione della testa del pupazzo di neve
    x_goal_head = [0.65 0.13 0.99]';


    % COSTRUZIONE DI XD, DXD, DDXD COMPLESSIVE

    % 1° passo: pos iniziale -> pos pre-saluto
    coeff_pol = five_degree_pol_system_solver(q0, 0, 0, q_goal_pre, 0, 0);
    [xd1, dxd1, ddxd1] = xd_solver(coeff_pol);

    % 2° passo: pos pre-saluto -> fine-corsa dx
    [xd2, dxd2, ddxd2] = xd_arco_circ(q_goal_pre, x_pre, x_goal_dx, tf, t_step);

    % 3° passo: fine-corsa dx -> fine-corsa sx
    [xd3, dxd3, ddxd3] = xd_arco_circ(q_goal_pre, x_goal_dx, x_goal_sx, 2*tf, t_step);

    % 4° passo: fine-corsa sx -> fine-corsa dx
    xd4 = xd3(:, end:-1:1);
    dxd4 = dxd3(:, end:-1:1);
    ddxd4 = ddxd3(:, end:-1:1);

    % 5° passo: fine-corsa dx -> fine-corsa sx
    xd5 = xd3;
    dxd5 = dxd3;
    ddxd5 = ddxd3;

    % 6° passo: fine-corsa sx -> pos pre-saluto
    [xd6, dxd6, ddxd6] = xd_arco_circ(q_goal_pre, x_goal_sx, x_pre, tf, t_step);

    % 7° passo: pos pre-saluto -> pos cappello
    [xd7, dxd7, ddxd7] = xd_rettilinea(x_pre, x_goal_cap, tf, t_step);

    % 8° passo: pos cappello -> pos testa pupazzo
    [xd8, dxd8, ddxd8] = xd_rettilinea(x_goal_cap, x_goal_head, tf, t_step);

    % 9° passo: pos testa pupazzo -> pos iniziale
    q_goal_head = inverse_kin_static(x_goal_head);
    coeff_pol = five_degree_pol_system_solver(q_goal_head, 0, 0, q_goal_fin, 0, 0);
    [xd9, dxd9, ddxd9] = xd_solver(coeff_pol);
    
    % XD COMPLESSIVA
    xd = xd1+ xd2 + xd3 + xd4 + xd5 + xd6 + xd7 + xd8 + xd9;

    % DXD COMPLESSIVA
    dxd = dxd1+ dxd2 + dxd3 + dxd4 + dxd5 + dxd6 + dxd7 + dxd8 + dxd9;

    % DDXD COMPLESSIVA
    ddxd = ddxd1+ ddxd2 + ddxd3 + ddxd4 + ddxd5 + ddxd6 + ddxd7 + ddxd8 + ddxd9;

end