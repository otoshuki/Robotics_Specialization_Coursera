function [ predictx, predicty, state, param ] = kalmanFilter( t, x, y, state, param, previous_t )
%UNTITLED Summary of this function goes here
%   Four dimensional state: position_x, position_y, velocity_x, velocity_y

    %% Place parameters like covarainces, etc. here:
    % P = eye(4)
    % R = eye(2)

    % Check if the first time running this function
    if previous_t<0
        state = [x, y, 0, 0];
        param.P = 0.1 * eye(4);
        predictx = x;
        predicty = y;
        return;
    end

    %% TODO: Add Kalman filter updates
    % As an example, here is a Naive estimate without a Kalman filter
    % You should replace this code
    %vx = (x - state(1)) / (t - previous_t);
    %vy = (y - state(2)) / (t - previous_t);
    % Predict 330ms into the future
    %predictx = x + vx * 0.330;
    %predicty = y + vy * 0.330;
    
    % State is a four dimensional element
    %state = [x, y, vx, vy];
    
    %My code
    %Dynamics Model
    dt = 0.033;
    A = [1,0,dt,0;0,1,0,dt;0,0,1,0;0,0,0,1];
    sigma_m = diag([0.5,0.2,0.1,0.4]);
    P = eye(4);
    %Measurement Model
    C = [1,0,0,0;0,1,0,0];
    sigma_o = diag([0.02,0.05]);
    R = eye(2);
    %Maximum A-Posterior Estimation
    P = A*param.P*A' + sigma_m;
    R = C*P*C' + sigma_o;
    K = P*C'*pinv(R+C*P*C'); %Kalman Gain
    next_state = A*state' + K*([x;y] - C*A*state');
    next_state = A*state' - K*C*A*state' + K*[x,y]';
    %Updates
    vx = next_state(3);
    vy = next_state(4);
    predictx = next_state(1) + vx*10*dt;
    predicty = next_state(2) + vy*10*dt;
    param.P = P - K*C*P; %Covariance update
    state = [x, y, vx, vy];
end
