function filter = initRefPathFilter(detection)
    filterCart = initcvekf(detection);
    
    % Convert the state to Frenet coordinates
    state = cartToFilterState(filterCart.State);
    
    % Define state covariance
    stateCov = blkdiag(5,100,4,5);
    
    % Define filter
    filter = trackingEKF(@stateTransFcn,@measurementFcn,state,...
    'StateTransitionJacobianFcn',@stateTransJacobianFcn,...
    'StateCovariance',stateCov,...
    'MeasurementNoise',detection.MeasurementNoise,...
    'HasAdditiveProcessNoise',false,...
    'ProcessNoise',blkdiag(10,1));
end