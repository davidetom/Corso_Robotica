function err = test_pose(sim, simulator, Q, dt)
% Confronta la posizione dell'end-effector calcolata con direct_kin con
% quella di l_EE_FRAME in CoppeliaSim, per ogni configurazione (riga) di Q.
% Se Q non e' dato usa 20 configurazioni di test; dt e' la pausa tra le pose.

    if nargin < 3 || isempty(Q),  Q = pose_di_test();  end
    if nargin < 4,                dt = 1;              end

    ee  = sim.getObject('/l_EE_FRAME');
    N   = size(Q, 1);
    err = zeros(N, 1);

    for k = 1:N
        q = Q(k, :);
        simulator.moveLeftArm(q);
        pause(dt);

        pc = cell2mat(sim.getObjectPosition(ee, sim.handle_world));
        p  = direct_kin(q, 'ZYZ');
        err(k) = norm(p(:) - pc(:));

        fprintf('%2d  q = [%s]  p_c = [%.3f %.3f %.3f]  errore = %.2f mm\n', ...
                k, sprintf('%6.2f', q), pc, 1000*err(k));
    end

    fprintf('Errore medio: %.2f mm, errore massimo: %.2f mm\n', 1000*mean(err), 1000*max(err));
end


function Q = pose_di_test()
% posa di riferimento, un giunto alla volta al 15% e all'85% dell'intervallo,
% 5 configurazioni casuali entro i limiti
    q_min = [-pi, -pi/2, -4.7124, -0.6792, -0.35, -1.9208, -pi/2];
    q_max = [ pi,  pi/2,  pi/2,    1.6708,  0.35, -1.2208,  pi/2];
    q_ref = [0, -pi/2, -pi/2, pi/2, 0, -pi/2, 0];

    Q = q_ref;
    for j = 1:7
        for f = [0.15, 0.85]
            q = q_ref;
            q(j) = q_min(j) + f * (q_max(j) - q_min(j));
            Q = [Q; q];
        end
    end

    rng(1);
    m = 0.1 * (q_max - q_min);
    for k = 1:5
        Q = [Q; q_min + m + rand(1, 7) .* (q_max - q_min - 2*m)];
    end
end
