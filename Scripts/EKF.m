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

x_est = robot.x;
P_est = robot.P

pos_robot = cell(N_laserscans);
cov_robot = cell(N_laserscans);


% Temporal cycle
for k = 1:N_laserscans
  N_feat_map = map.size();
  F_X = eye(2*N_feat_map + 3);
  F_X(1:3,1:3) = robot.JF_x();
  F_N = zeros(2*N_feat_map + 3, 3);
  F_N(1:3,1:3) = robot.JF_n();
  N = odometries(k).Q;
  
  
  % Prediction
  x_est(1:3) = robot.update_step(odometries(k))
  P_est = F_X*P*F_X' + F_N*N*F_N';

  
  % Update
  [z, H_X, R] = map.compute_innovation(robot, laserscans{k}.observations);
  S = H_X*P_est*H_X' + R;
  W = P_est*H_X/S;
  x_est = x_est + W*z;
  P_est = P_est - W*H_X*P_est;

  pos_robot(end+1,:) = x_est;
  cov_robot(end+1,:) = P_est;


end