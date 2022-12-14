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

tmp = 0;
tmp2 = 0;

T_limit = N_laserscans;
P_est_norm = zeros(T_limit, 1);

disp('Starting the cycle')
% Temporal cycle
for k = 1:T_limit

  fprintf('================================> Iteration %6d <================================\n', k);
  fprintf('Current map size: %d\n', map.size());
   
  check_covariance_matrix(P_est, 'Iteration start');

  N_feat_map    = map.size();
  F_X           = eye(2*N_feat_map + 3);
  F_X(1:3,1:3)  = robot.JF_x(odometries{1,k});
  F_N           = zeros(2*N_feat_map + 3, 3);
  F_N(1:3,1:3)  = robot.JF_n();
  N             = odometries{k}.Q;
  
  
  % Prediction
  fprintf('Prediction...');
  x_est(1:3) = robot.update_step(odometries{k});
  P_est = F_X*P_est*F_X' + F_N*N*F_N';
  fprintf('Done!\n');
  check_covariance_matrix(P_est, 'After prediction');

  fprintf('Copying states from EKF to map...');
  robot.P = P_est(1:3, 1:3);
  for i = 1:map.size()
    map.landmark_vector(i).x = x_est(3 + 2*i - 1:3 + 2*i);
    map.landmark_vector(i).P = P_est(3 + 2*i - 1:3 + 2*i, 3 + 2*i - 1:3 + 2*i);
  end
  fprintf('Done!\n');
  
  tmp = eig(P_est);
  tmp2 = diag(P_est);
  
  % Update if the map is not empty
  if map.size() > 0

    fprintf('Performing an update step ');
    [z, H_X, R] = map.compute_innovation(robot, laserscans{k}.observations);
    fprintf('using %d observations...', round(length(z)/2));
    S = H_X*P_est*H_X' + R;
    W = P_est*H_X'*inv(S);
    x_est = x_est + W*z;
    % P_est = P_est - W*H_X*P_est;
    P_est = P_est - W*S*W';
    % P_est = (eye(size(P_est)) - W*H_X)*P_est;
    fprintf('Done!\n');
  else
    fprintf('Empty map, no update step necessary!\n');
  end
  

  check_covariance_matrix(P_est, 'After update')  

  % Update the map

  fprintf('Copying states from EKF to map...');
  robot.x = x_est(1:3);
  robot.P = P_est(1:3, 1:3);
  P_est_norm(k) = norm(robot.P);
  
  for i = 1:map.size()
    map.landmark_vector(i).x = x_est(3 + 2*i - 1:3 + 2*i);
    map.landmark_vector(i).P = P_est(3 + 2*i - 1:3 + 2*i, 3 + 2*i - 1:3 + 2*i);

    check_covariance_matrix(map.landmark_vector(i).P, 'Copying landmark after update')
  end
  fprintf('Done!\n');

  fprintf('Performing map update...');
  observation_to_add = laserscans{k}.observations(2:end-1);
  new_features = map.update_map(robot, observation_to_add);
  fprintf('found %d new features\n', length(new_features));

  for i = 1:length(new_features)
    fprintf('Adding new feature #%2d (observation %d)... ', i, new_features(i));

    landmark_index = new_features(i);
    obs = observation_to_add{landmark_index};
    landmark = Landmark(robot, obs);

    P_LL      = landmark.P;                         % eq (35)
    check_covariance_matrix(P_LL, 'Stacking a new landmark in P_LL');
    P_Rx      = P_est(1:3, :);                      % eq (6)
    [JG_R, ~] = landmark.compute_jacobians(robot, obs);
    P_Lx      = JG_R*P_Rx;                          % eq (36)

    x_est = [x_est; landmark.x];                    % eq (37)
    P_est = [P_est, P_Lx';                          % eq (38)
             P_Lx,  P_LL];
    fprintf('Done!\n');

    check_covariance_matrix(P_est, 'Stacking a new landmark');
  end
  
  figure(2), clf;
  plot(map, length(new_features));

  figure(3), clf;
  plot(laserscans{k});
  
  figure(4), clf;
  plot(P_est_norm);

end




function check_covariance_matrix(P, text)
  if ~all(eig(P) >= 0)
    if nargin == 2
      fprintf('\n\nCOVARIANCE ERROR: %s\n\n', text);
    end
    error('The covariance matrix of the observation is not positive definite');
  end
end