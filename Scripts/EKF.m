% The model of the analized problem is:
%     
%     x(k+1) = fk(x(k),u(k),nu(k)) -> nu is the noise in the model
%     z(k) = hk(x(k),epsilon(k)) -> epsilon is the noise on the measures
% 
% The Ekf is divided into steps: 
%     - predicion:
%         x_est(k+1) = fk(x_est(k), u(k))
%         P_est(k+1) = A(k)P_est(k)A(k)' + G(k)Q(k)G(k)'
% 
%         where:
%         A(k) is the jacobian of fk(x,u,nu) w.r.t the state e and evaluated in the x_est(k) and u(k) with nu = 0
%         G(k) is the jacobian of fk(x,u,nu) w.r.t nu e and evaluated in the x_est(k) and u(k) with nu = 0
%      
%      - update:
%         S(k+1) = H(k+1)P_est(k+1)H(k+1)' + R(k+1)
%         W(k+1) = P_est(k+1)H(k+1)'/S(k+1)
%         x_est(k+1) = x_est(k+1) + W(k+1)(z(k+1) - hk+1(x_est(k+1)))
%         P_est(k+1) = (I - W(k+1)H(k+1))P_est(k+1)
% 
%         where:
%         H(k+1) is the jacobian of hk+1(x,epsilon) w.r.t the state e and evaluated in the x_est(k+1) and epsilon = 0

robot = Robot();
map   = Map();

x_est = zeros(3, 1);
P_est = zeros(3, 3);

pos_robot = cell(N_laserscans,1);
cov_robot = cell(N_laserscans,1);

disp('Starting the cycle')
% Temporal cycle
for k = 1:50 %N_laserscans

  disp(['iteration: ',num2str(k)])

  N_feat_map    = map.size();
  F_X           = eye(2*N_feat_map + 3);
  F_X(1:3,1:3)  = robot.JF_x();
  F_N           = zeros(2*N_feat_map + 3, 3);
  F_N(1:3,1:3)  = robot.JF_n();
  N             = odometries{k}.Q;
  
  
  % Prediction
  x_est(1:3) = robot.update_step(odometries{k});
  P_est = F_X*P_est*F_X' + F_N*N*F_N';
  disp('Done prediction')

  robot.P = P_est(1:3, 1:3);
  
  % Update if the map is not empty
  if map.size() > 0
    [z, H_X, R] = map.compute_innovation(robot, laserscans{k}.observations);
    S = H_X*P_est*H_X' + R;
    W = P_est*H_X'*inv(S);
    x_est = x_est + W*z
    % P_est = P_est - W*H_X*P_est;
    P_est = P_est - W*S*W';

    robot.x = x_est(1:3);
    robot.P = P_est(1:3, 1:3);

    
    disp('Done update')
  else
    disp('Map is empty: building it...');
  end

  % Storing the history of the trajectory
  pos_robot{k} = x_est(1:3,:);
  cov_robot{k} = P_est;

  % Update the map
  for i = 1:map.size()
    map.landmark_vector(i).x = x_est(3 + 2*i - 1:3 + 2*i);
    map.landmark_vector(i).P = P_est(3 + 2*i - 1:3 + 2*i, 3 + 2*i - 1:3 + 2*i);
    disp('Creating landmark vector')
  end

  new_features = map.update_map(robot, laserscans{k}.observations);
  disp('new_features: ')
  disp(new_features)

  for i = 1:length(new_features)
    disp('Updating the whole state and covariance')
    landmark = new_features(i);

    P_LL      = landmark.P;                         % eq (35)
    P_Rx      = P_est(1:3, :);                      % eq (6)
    [JG_R, ~] = landmark.compute_jacobians(robot, Observation(landmark.x));
    P_Lx      = JG_R*P_Rx;                          % eq (36)

    x_est = [x_est; landmark.x];                    % eq (37)
    P_est = [P_est, P_Lx';                          % eq (38)
             P_Lx,  P_LL];
  end
  

end