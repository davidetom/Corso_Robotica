function [X,dX,ddX,Q,dQ,ddQ] = inverse_kin(q0, xd, dxd, ddxd)

    dT = 0.001;
    q = q0(:); dq = zeros(7, 1);
    N = size(xd, 2);
    
    % array per salvare l'evoluzione del controllo
    X = zeros(N, 6);
    dX = zeros(N, 6);
    ddX = zeros(N, 6);
    Q = zeros(N, 7);
    dQ = zeros(N, 7);
    ddQ = zeros(N, 7);

    % guadagni PD (suggerimento per smorzamento critico se necessario: es. Kp=100, Kd=20)
    Kp = 10 * eye(6);
    Kd = 2 * eye(6);

    % guadagno proiezione nel nullo
    Kn = 5 * eye(7);

    h = 1e-6;

    for i = 1:N

        % campione desiderato all'istante i-esimo
        xd_i   = xd(:, i);
        dxd_i  = dxd(:, i);
        ddxd_i = ddxd(:, i);

        [p, phi, ~, ~] = direct_kin(q, 'ZYZ');
        x = [p ; phi];

        Ja = analytical_jacobian(q, 'ZYZ');
        dx = Ja*dq;

        e = xd_i - x;
        e(4:6) = atan2(sin(e(4:6)), cos(e(4:6)));

        term1 = ddxd_i + Kd * (dxd_i - dx) + Kp * e;
        ddx = term1;
        
        Ja_next = analytical_jacobian(q + h * dq, 'ZYZ');
        dJa = (Ja_next - Ja) / h;                    
        term2 = dJa * dq; 

        Ja_pinv = pinv(Ja);
        
        % inversione semplice a norma minima
        ddq = Ja_pinv * (term1 - term2);

        % aggiunta della proiezione nel nullo
        t = (i - 1) * dT;
        [~, grad_f] = cost_function(q, t, Ja);
        g = - grad_f;
        if i == 1
            dg = zeros(7,1);
        else
            dg = (g - g_prev) / dT;
        end
        g_prev = g;
        dJa_pinv = (pinv(Ja_next) - Ja_pinv) / h;
        n_projection = (eye(7) - Ja_pinv * Ja);
        de_n = n_projection * (g - dq);

        phi_n = n_projection * (dg + Kn * de_n) - ...
                (Ja_pinv * dJa * Ja_pinv  + dJa_pinv) * Ja * (g - dq);
        
        ddq = ddq + phi_n;
        
        % aggiornamenti
        X(i,:) = x;
        dX(i,:) = dx;
        ddX(i,:) = ddx;
        Q(i,:) = q;
        dQ(i,:) = dq;
        ddQ(i,:) = ddq;
    
        % integrazione Euler forward
        dq = dq + ddq * dT;
        q = q + dq * dT;
       
    end
end