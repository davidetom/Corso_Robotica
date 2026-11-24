% Main script for Homework2
clear all 
close all
addpath(genpath('/Applications/coppeliaSim.app/Contents/Resources/programming/zmqRemoteApi/clients/matlab'));

client = RemoteAPIClient();
sim = client.getObject('sim');

% Joint space configuration
qDH = [0, -pi/3, -pi/2, 0, 0, 0.6981 - pi, 0];

% Joint space limits
qMinDH = [-3.1416   -1.5708   -4.7124   -0.6792   -1.3963    -2.9671   -1.5708]'; %rad x7
qMaxDH = [3.1416    1.5708    1.5708   1.6708    1.3963    -0.1745    1.5708]'; %rad x7

[xd, dxd, ddxd] = trajectory();
[q, dq, ddq] = inverse_kin(qDH, xd, dxd, ddxd);

% Modalita' sincrona (stepping): applica tutti e 7 i giunti nello stesso frame
% evitando gli scatti dovuti alla latenza ZMQ.
sim.setStepping(true);
simulator = SimProjectUtils(sim);

N = size(q, 1);
dT = 0.001;
step = 25; % 25 ms = dt di default di CoppeliaSim (40 FPS)

t_start = tic;
for k = 1:step:N
    simulator.moveLeftArm(q(k, :));
    sim.step();

    if k == 6501
        simulator.graspObject();
    elseif k == 8001
        simulator.releaseObject();
    end

    % Sincronizza al tempo reale senza accumulare ritardo
    t_wait = (k - 1) * dT - toc(t_start);
    if t_wait > 0
        pause(t_wait);
    end
end
simulator.moveLeftArm(q(end, :));
sim.step();


%% Plot dei risultati
plot_results(xd, dxd, ddxd, q, dq, ddq, qMinDH, qMaxDH, dT);

% Funzione locale di plot
function plot_results(xd, dxd, ddxd, q, dq, ddq, qMinDH, qMaxDH, dT)
    N = size(q, 1);
    t = (0:N-1) * dT;

    % Ricostruzione cinematica diretta di x, dx, ddx in spazio operativo
    x   = zeros(N, 6);
    dx  = zeros(N, 6);
    ddx = zeros(N, 6);
    h   = 1e-6;

    for i = 1:N
        qi   = q(i, :)';
        dqi  = dq(i, :)';
        ddqi = ddq(i, :)';

        [p, phi, ~, ~] = direct_kin(qi, 'ZYZ');
        x(i, :) = [p; phi]';

        Ja = analytical_jacobian(qi, 'ZYZ');
        dx(i, :) = (Ja * dqi)';

        Ja_next = analytical_jacobian(qi + h * dqi, 'ZYZ');
        dJa = (Ja_next - Ja) / h;
        ddx(i, :) = (Ja * ddqi + dJa * dqi)';
    end

    % Porta i riferimenti in formato N x 6 per coerenza con x, dx, ddx
    xd_t   = xd';
    dxd_t  = dxd';
    ddxd_t = ddxd';

    % Errori in spazio operativo su posa, velocita' e accelerazione
    e_pos   = xd_t(:, 1:3) - x(:, 1:3);
    e_ori   = atan2(sin(xd_t(:, 4:6) - x(:, 4:6)), cos(xd_t(:, 4:6) - x(:, 4:6)));
    de_pos  = dxd_t(:, 1:3) - dx(:, 1:3);
    de_ori  = dxd_t(:, 4:6) - dx(:, 4:6);
    dde_pos = ddxd_t(:, 1:3) - ddx(:, 1:3);
    dde_ori = ddxd_t(:, 4:6) - ddx(:, 4:6);

    % --- Figura 1: Posizione, orientamento ed errori in spazio operativo ---
    figure('Name', 'Spazio Operativo - Posizione, Orientamento ed Errori');

    subplot(3, 1, 1);
    plot(t, x(:, 1:3), 'LineWidth', 1.2); hold on;
    plot(t, xd_t(:, 1:3), '--', 'LineWidth', 1.0);
    grid on; ylabel('Posizione [m]');
    title('Posizione End-Effector (continua: effettiva, tratteggiata: desiderata)');
    legend('x', 'y', 'z', 'x_d', 'y_d', 'z_d', 'Location', 'bestoutside');

    subplot(3, 1, 2);
    plot(t, unwrap(x(:, 4:6), [], 1), 'LineWidth', 1.2); hold on;
    plot(t, xd_t(:, 4:6), '--', 'LineWidth', 1.0);
    grid on; ylabel('Angoli ZYZ [rad]');
    title('Orientamento End-Effector (continua: effettiva, tratteggiata: desiderata)');
    legend('\gamma', '\theta', '\psi', '\gamma_d', '\theta_d', '\psi_d', 'Location', 'bestoutside');

    subplot(3, 1, 3);
    yyaxis left;
    plot(t, e_pos, 'LineWidth', 1.0);
    ylabel('Errore pos [m]');
    yyaxis right;
    plot(t, e_ori, '--', 'LineWidth', 1.0);
    ylabel('Errore ori [rad]');
    grid on; xlabel('Tempo [s]');
    title('Errori di inseguimento in spazio operativo');
    legend('e_x', 'e_y', 'e_z', 'e_\gamma', 'e_\theta', 'e_\psi', 'Location', 'bestoutside');

    % --- Figura 2: Velocita' e accelerazioni in spazio operativo ---
    figure('Name', 'Spazio Operativo - Velocita e Accelerazioni');

    subplot(2, 2, 1);
    plot(t, dx(:, 1:3), 'LineWidth', 1.2); hold on;
    plot(t, dxd_t(:, 1:3), '--', 'LineWidth', 1.0);
    grid on; ylabel('dx [m/s]'); title('Velocita lineare');
    legend('dx', 'dy', 'dz', 'dx_d', 'dy_d', 'dz_d', 'Location', 'best');

    subplot(2, 2, 2);
    plot(t, dx(:, 4:6), 'LineWidth', 1.2); hold on;
    plot(t, dxd_t(:, 4:6), '--', 'LineWidth', 1.0);
    grid on; ylabel('d\phi [rad/s]'); title('Velocita angoli ZYZ');
    legend('d\gamma', 'd\theta', 'd\psi', 'd\gamma_d', 'd\theta_d', 'd\psi_d', 'Location', 'best');

    subplot(2, 2, 3);
    plot(t, ddx(:, 1:3), 'LineWidth', 1.2); hold on;
    plot(t, ddxd_t(:, 1:3), '--', 'LineWidth', 1.0);
    grid on; xlabel('Tempo [s]'); ylabel('ddx [m/s^2]'); title('Accelerazione lineare');
    legend('ddx', 'ddy', 'ddz', 'ddx_d', 'ddy_d', 'ddz_d', 'Location', 'best');

    subplot(2, 2, 4);
    plot(t, ddx(:, 4:6), 'LineWidth', 1.2); hold on;
    plot(t, ddxd_t(:, 4:6), '--', 'LineWidth', 1.0);
    grid on; xlabel('Tempo [s]'); ylabel('dd\phi [rad/s^2]'); title('Accelerazione angoli ZYZ');
    legend('dd\gamma', 'dd\theta', 'dd\psi', 'dd\gamma_d', 'dd\theta_d', 'dd\psi_d', 'Location', 'best');

    % --- Figura 3: Errori in spazio operativo su posa, velocita' e accelerazione ---
    figure('Name', 'Spazio Operativo - Errori su Posa, Velocita e Accelerazione');

    subplot(3, 2, 1);
    plot(t, e_pos * 1e3, 'LineWidth', 1.1);
    grid on; ylabel('e_p [mm]'); title('Errore di posizione (p_d - p)');
    legend('e_x', 'e_y', 'e_z', 'Location', 'best');

    subplot(3, 2, 2);
    plot(t, e_ori * 1e3, 'LineWidth', 1.1);
    grid on; ylabel('e_\phi [mrad]'); title('Errore di orientamento (\phi_d - \phi)');
    legend('e_\gamma', 'e_\theta', 'e_\psi', 'Location', 'best');

    subplot(3, 2, 3);
    plot(t, de_pos * 1e3, 'LineWidth', 1.1);
    grid on; ylabel('de_p [mm/s]'); title('Errore di velocita lineare (dp_d - dp)');
    legend('de_x', 'de_y', 'de_z', 'Location', 'best');

    subplot(3, 2, 4);
    plot(t, de_ori * 1e3, 'LineWidth', 1.1);
    grid on; ylabel('de_\phi [mrad/s]'); title('Errore di velocita angoli ZYZ (d\phi_d - d\phi)');
    legend('de_\gamma', 'de_\theta', 'de_\psi', 'Location', 'best');

    subplot(3, 2, 5);
    plot(t, dde_pos, 'LineWidth', 1.1);
    grid on; xlabel('Tempo [s]'); ylabel('dde_p [m/s^2]'); title('Errore di accelerazione lineare (ddp_d - ddp)');
    legend('dde_x', 'dde_y', 'dde_z', 'Location', 'best');

    subplot(3, 2, 6);
    plot(t, dde_ori, 'LineWidth', 1.1);
    grid on; xlabel('Tempo [s]'); ylabel('dde_\phi [rad/s^2]'); title('Errore di accelerazione angoli ZYZ (dd\phi_d - dd\phi)');
    legend('dde_\gamma', 'dde_\theta', 'dde_\psi', 'Location', 'best');

    % --- Figura 4: Posizioni, velocita' e accelerazioni ai giunti ---
    figure('Name', 'Spazio dei Giunti - q, dq, ddq');
    joint_labels = {'q_1', 'q_2', 'q_3', 'q_4', 'q_5', 'q_6', 'q_7'};

    subplot(3, 1, 1);
    plot(t, q, 'LineWidth', 1.2);
    grid on; ylabel('q [rad]'); title('Posizioni dei giunti');
    legend(joint_labels, 'Location', 'bestoutside');

    subplot(3, 1, 2);
    plot(t, dq, 'LineWidth', 1.2);
    grid on; ylabel('dq [rad/s]'); title('Velocita dei giunti');
    legend(joint_labels, 'Location', 'bestoutside');

    subplot(3, 1, 3);
    plot(t, ddq, 'LineWidth', 1.2);
    grid on; xlabel('Tempo [s]'); ylabel('ddq [rad/s^2]'); title('Accelerazioni dei giunti');
    legend(joint_labels, 'Location', 'bestoutside');

    % --- Figura 5: Confronto giunti con i limiti qMinDH / qMaxDH ---
    figure('Name', 'Verifica Limiti di Giunto');
    for j = 1:7
        subplot(4, 2, j);
        plot(t, q(:, j), 'b', 'LineWidth', 1.2); hold on;
        yline(qMinDH(j), 'r--', 'LineWidth', 1.0);
        yline(qMaxDH(j), 'r--', 'LineWidth', 1.0);
        grid on;
        title(sprintf('Giunto %d', j));
        ylabel('rad');
        if j >= 6
            xlabel('Tempo [s]');
        end
    end

    % --- Figura 6: Analisi singolarita' (det(Ja*Ja'), manipolabilita', sigma_min) ---
    idx_s = 1:10:N; % sottocampionamento (ogni 10 ms) per velocizzare il plot
    t_s = t(idx_s);
    det_JJt = zeros(size(idx_s));
    w_manip = zeros(size(idx_s));
    sigma_min = zeros(size(idx_s));

    for m = 1:length(idx_s)
        Ja_m = analytical_jacobian(q(idx_s(m), :)', 'ZYZ');
        det_JJt(m) = det(Ja_m * Ja_m');
        w_manip(m) = sqrt(max(det_JJt(m), 0));
        sv = svd(Ja_m);
        sigma_min(m) = sv(end);
    end

    figure('Name', 'Analisi Singolarita (Jacobiano Analitico)');

    subplot(3, 1, 1);
    plot(t_s, det_JJt, 'LineWidth', 1.2);
    grid on; ylabel('det(J_A J_A^T)');
    title('Determinante di J_A J_A^T (si annulla in singolarita)');

    subplot(3, 1, 2);
    plot(t_s, w_manip, 'LineWidth', 1.2);
    grid on; ylabel('w(q)');
    title('Indice di manipolabilita w(q) = sqrt(det(J_A J_A^T))');

    subplot(3, 1, 3);
    plot(t_s, sigma_min, 'LineWidth', 1.2);
    grid on; xlabel('Tempo [s]'); ylabel('\sigma_{min}(J_A)');
    title('Valore singolare minimo di J_A');

    % --- Figura 7: Traiettoria 3D dell'End-Effector ---
    figure('Name', 'Traiettoria 3D End-Effector');
    plot3(xd_t(:, 1), xd_t(:, 2), xd_t(:, 3), '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2); hold on;
    plot3(x(1:1001, 1),    x(1:1001, 2),    x(1:1001, 3),    'Color', [0.00 0.45 0.74], 'LineWidth', 1.8);
    plot3(x(1001:5001, 1), x(1001:5001, 2), x(1001:5001, 3), 'Color', [0.47 0.67 0.19], 'LineWidth', 1.8);
    plot3(x(5001:6501, 1), x(5001:6501, 2), x(5001:6501, 3), 'Color', [0.85 0.33 0.10], 'LineWidth', 1.8);
    plot3(x(6501:8001, 1), x(6501:8001, 2), x(6501:8001, 3), 'Color', [0.49 0.18 0.56], 'LineWidth', 1.8);
    plot3(x(8001:end, 1),  x(8001:end, 2),  x(8001:end, 3),  'Color', [0.30 0.75 0.93], 'LineWidth', 1.8);

    % Punti chiave della traiettoria
    plot3(xd_t(1, 1),    xd_t(1, 2),    xd_t(1, 3),    'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 7);
    plot3(xd_t(1001, 1), xd_t(1001, 2), xd_t(1001, 3), 'gs', 'MarkerFaceColor', 'g', 'MarkerSize', 7);
    plot3(xd_t(6501, 1), xd_t(6501, 2), xd_t(6501, 3), 'r^', 'MarkerFaceColor', 'r', 'MarkerSize', 8);
    plot3(xd_t(8001, 1), xd_t(8001, 2), xd_t(8001, 3), 'md', 'MarkerFaceColor', 'm', 'MarkerSize', 8);

    grid on; axis equal;
    xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
    title('Traiettoria 3D End-Effector nello spazio operativo');
    legend('Desiderata x_d', '1: Iniziale \rightarrow Pre-saluto', '2-6: Saluto', ...
           '7: Pre-saluto \rightarrow Cappello', '8: Cappello \rightarrow Testa', ...
           '9: Testa \rightarrow Iniziale', ...
           'p_0 (iniziale)', 'p_{pre} (saluto)', 'p_{hat} (presa)', 'p_{d,hat} (rilascio)', ...
           'Location', 'bestoutside');
    view(130, 22);
end
