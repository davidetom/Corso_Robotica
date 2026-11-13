function [J] = eval_geometric_jacobian(q)
% Valuta numericamente lo Jacobiano geometrico

% Carica J_sym e theta_param salvati offline
persistent J_sym theta_param
if isempty(J_sym)
    data = load('jacobiano_robot.mat', 'J_sym', 'theta_param');
    J_sym = data.J_sym;
    theta_param = data.theta_param;
end

% Sostituisce i valori attuali dei theta e converte in double
J = double(subs(J_sym, theta_param, q));

end