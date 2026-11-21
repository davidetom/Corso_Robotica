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

        term1 = ddxd_i + Kd*(dxd_i - dx) + Kp*e;
        ddx = term1;

        dJa = (analytical_jacobian(q + h * dq, 'ZYZ') - Ja) / h;                    
        term2 = dJa * dq; 
        
        % inversione semplice a norma minima (proiezione nel nullo da aggiungere)
        ddq = pinv(Ja) * (term1 - term2);
        
        % aggiornamenti
        X(i,:) = x;
        dX(i,:) = dx;
        ddX(i,:) = ddx;
        Q(i,:) = q;
        dQ(i,:) = dq;
        ddQ(i,:) = ddq;
    
        % integrazione Euler forward
        dq = dq + ddq*dT;
        q = q + dq*dT;
       
    end
end