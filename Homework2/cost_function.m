function [f, grad_f] = cost_function(q, t, Ja)
% Funzione di costo complessiva f(q, t) e suo gradiente per
% la proiezione nello spazio nullo

    q = q(:);
    n = length(q);

    if nargin < 2 || isempty(t)
        t = 0;
    end

    % ======================== PARAMETRI =================================
    % Pesi dei tre contributi (0 per disattivare un termine)
    k_lim  = 100.0;   % mantenimento limiti di giunto
    k_saluto = 1.0;   % mantenimento postura durante il saluto
    k_sing = 20.0;   % evitamento singolarità

    % Pesi per ogni giunto nella postura di saluto:
    % in trajectory.m durante il saluto varia il giunto 3, mentre 1, 2, 4
    % restano fissi (spalla e gomito).
    W_saluto = diag([5.0, 5.0, 0.0, 5.0, 0.5, 0.5, 0.5]);

    % Finestra temporale del saluto (passi 2 -> 6 in trajectory.m: t in [3, 15] s)
    t_wave_start = 1.0;
    t_wave_end   = 5.0;
    t_ramp       = 0.25;   % durata rampa liscia

    % Passo per differenze finite del termine di singolarita'
    delta   = 1e-5;      % variazione per calcolo delle derivate parziali
    epsilon = 1e-8;      % per evitare divisione per 0 in singolarita'
    % ====================================================================

    % Limiti di giunto
    qMinDH = [-3.1416; -1.5708; -4.7124; -0.6792; -1.3963; -2.9671; -1.5708];
    qMaxDH = [ 3.1416;  1.5708;  1.5708;  1.6708;  1.3963; -0.1745;  1.5708];
    q_mean  = 0.5 * (qMinDH + qMaxDH);
    q_span = qMaxDH - qMinDH;

    % Configurazione di riferimento pre-saluto (corrisponde a q_goal_pre in trajectory.m)
    q_ref = [-pi/3; -1.42; -pi/2; pi/20; 0; -pi/2; 0];

    % ======================== FUNZIONI DI COSTO =========================
    % 1) Evitamento limiti di giunto: f_lim(q)
    % f_lim = (k_lim / (2*n)) * sum( ((q_j - q_mean_j) / (qMax_j - qMin_j))^2 )
    if k_lim > 0
        diff_norm = (q - q_mean) ./ q_span;
        f_lim     = (k_lim / (2 * n)) * sum(diff_norm.^2);
        grad_lim  = (k_lim / n) * (diff_norm ./ q_span);
    else
        f_lim    = 0;
        grad_lim = zeros(n, 1);
    end

    % 2) Mantenimento postura durante il saluto: f_saluto(q, t)
    % f_saluto = k_saluto * 0.5 * (q - q_ref)' * W_saluto * (q - q_ref) * alpha(t)
    if k_saluto > 0
        alpha     = smooth_window(t, t_wave_start, t_wave_end, t_ramp);
        dq_ref    = q - q_ref;
        f_saluto    = k_saluto * 0.5 * (dq_ref' * W_saluto * dq_ref) * alpha;
        grad_saluto = k_saluto * (W_saluto * dq_ref) * alpha;
    else
        f_saluto    = 0;
        grad_saluto = zeros(n, 1);
    end

    % 3) Evitamento singolarita': f_sing(q) = -k_sing * w(q)
    % Massimizza la manipolabilita' di Yoshikawa citata nel paper:
    % w(q) = sqrt(det(Ja * Ja'))
    if k_sing > 0
        w = sqrt(max(det(Ja * Ja'), 0) + epsilon);
        f_sing = -k_sing * w;
        
        % Calcolo del gradiente con derivata parziale numerica 
        % per ciascun q_i
        grad_sing = zeros(n, 1);
        for j = 1:n
            q_delta    = q;
            q_delta(j) = q_delta(j) + delta;
            Ja_delta   = analytical_jacobian(q_delta, 'ZYZ');
            w_delta    = sqrt(max(det(Ja_delta * Ja_delta'), 0) + epsilon);
            grad_sing(j) = -k_sing * (w_delta - w) / delta;
        end
    else
        f_sing    = 0;
        grad_sing = zeros(n, 1);
    end

    % Somma totale
    f      = f_lim + f_saluto + f_sing;
    grad_f = grad_lim + grad_saluto + grad_sing;
end


%----------------------------------------------------------------


% Funzione ausiliaria: finestra temporale liscia C^1 in [0, 1] 
%       _________
% _____/         \_____

function alpha = smooth_window(t, t_start, t_end, t_ramp)
    if t <= (t_start - t_ramp) || t >= (t_end + t_ramp)
        alpha = 0;
    elseif t < t_start
        s = (t - (t_start - t_ramp)) / t_ramp;
        alpha = 0.5 * (1 - cos(pi * s));
    elseif t <= t_end
        alpha = 1;
    else
        s = (t - t_end) / t_ramp;
        alpha = 0.5 * (1 + cos(pi * s));
    end
end
