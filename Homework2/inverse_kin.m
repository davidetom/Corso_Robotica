function [X,dX,ddX,Q,dQ,ddQ] = inverse_kin(q0, xd, dxd, ddxd, Tmax)
    
    dT = 0.001;
    N = round(Tmax / dT);
    q = q0(:); dq = zeros(7, 1);
    xd = xd(:); dxd = dxd(:); ddxd = ddxd(:);
    
    % Array per salvare l'evoluzione del controllo
    X = zeros(N, 6);
    dX = zeros(N, 6);
    ddX = zeros(N, 6);
    Q = zeros(N, 7);
    dQ = zeros(N, 7);
    ddQ = zeros(N, 7);

    Kp = 10 * eye(6);
    Kd = 2 * eye(6);
    h = 1e-6;

    for i = 1:N

        [p, phi, ~, ~] = direct_kin(q, 'ZYZ');
        x = [p ; phi];

        Ja = analytical_jacobian(q, 'ZYZ');
        dx = Ja*dq;

        term1 = ddxd + Kd*(dxd - dx) + Kp*(xd -x);
        ddx = term1;

        dJa = (analytical_jacobian(q + h * dq, 'ZYZ') - Ja) / h;                    
        term2 = dJa * dq; 
        
        ddq = pinv(Ja) * (term1 - term2);
        
        % aggiornamenti
        X(i,:) = x;
        dX(i,:) = dx;
        ddX(i,:) = ddx;
        Q(i,:) = q;
        dQ(i,:) = dq;
        ddQ(i,:) = ddq;
    
        % Integrazione Euler forward
        dq = dq + ddq*dT;
        q = q + dq*dT;
       
    end
end