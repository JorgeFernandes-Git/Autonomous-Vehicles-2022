function [dfdx, dfdw] = stateTransJacobianFcn(~, ~, dT)
    % Esta função define o jabociano da função de transição de estado, o jacobiano é usado no Extender Kalman Filter, o EKF trabalha em torno de um ponto de linearização local usando o jacobiano calculado nesse ponto para definir a próxima posição.
    tau = 3;
    dfdx = [1 dT 0 0;0 1 0 0;0 0 1 tau*(1 - exp(-dT/tau));0 0 0 exp(-dT/tau)];
    dfdw = [dT^2/2 0;dT 0;0 dT^2/2;0 dT];
end

