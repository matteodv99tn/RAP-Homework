% Save data od the map
data_map = fopen('SAVED_DATA\map_landmark_data.txt','wt');

for i = 1:length(map.landmark_vector)
    
    x = map.landmark_vector(i).x(1);
    y = map.landmark_vector(i).x(2);
    
    fprintf(data_map,'%f %f\n',x,y);
    
end
fclose('all');

%% Save estimated position
robot_est = fopen('SAVED_DATA\position_estimate.txt','wt');

for i = 1:length(pos_robot)
    
    x = pos_robot{i}(1);
    y = pos_robot{i}(2);
    t = pos_robot{i}(3);
    
    fprintf(robot_est,'%f %f %f\n',x,y,t);
    
end
fclose('all');


%% Load data
loaded_data_map  = readmatrix('SAVED_DATA\map_landmark_data.txt');
loaded_robot  = readmatrix('SAVED_DATA\position_estimate.txt');

loaded_landmark_vector = cell(size(loaded_data_map,1),1);
loaded_robot_estimate = cell(size(loaded_robot,1),1);

for i=1:size(loaded_data_map,1)
    loaded_landmark_vector{i}.x(1) = loaded_data_map(i,1);
    loaded_landmark_vector{i}.x(2) = loaded_data_map(i,2);
end

for i = 1:size(loaded_robot,1)
    loaded_robot_estimate{i}.x(1) = loaded_robot(i,1);
    loaded_robot_estimate{i}.x(2) = loaded_robot(i,2);
    loaded_robot_estimate{i}.x(3) = loaded_robot(i,3);
end

%% Plot
figure(8)
for i = 1:length(loaded_landmark_vector)
    mapx(i) = loaded_landmark_vector{i}.x(1);
    mapy(i) = loaded_landmark_vector{i}.x(2);
end
for i = 1:50:length(loaded_robot_estimate)
    robx(i) = loaded_robot_estimate{i}.x(1);
    roby(i) = loaded_robot_estimate{i}.x(2);
end
plot(mapx,mapy,'*b');
hold on
plot(robx,roby,'.r');
hold on
plot(robx(1),roby(1),'og','MarkerSize',10,'LineWidth',2);
hold on
plot(robx(end),roby(end),'ok','MarkerSize',10,'LineWidth',2);
title('Estimated Map and Position');
xlabel('x [m]');
ylabel('y [m]');
legend('Map','Trajectory','Start','End','Location','Best');

hold off


