% Save data od the map
data_map = fopen('SAVED_DATA\map_landmark_data.txt','wt');

for i = 1:length(map.landmark_vector)
    
    x = map.landmark_vector(i).x(1);
    y = map.landmark_vector(i).x(2);
    
    fprintf(data_map,'%f %f\n',x,y);
    
end
fclose('all');
%% Save covariance of the map
data_map = fopen('SAVED_DATA\map_landmark_covariance.txt','wt');

for i = 1:length(map.landmark_vector)
    
    p11 = map.landmark_vector(i).P(1,1);
    p12 = map.landmark_vector(i).P(1,2);
    p21 = map.landmark_vector(i).P(2,1);
    p22 = map.landmark_vector(i).P(2,2);
    
    fprintf(data_map,'%f\n%f\n%f\n%f\n',p11,p12,p21,p22);
    
end
fclose('all');

%% Load data
loaded_data_map  = readmatrix('SAVED_DATA\map_landmark_data.txt');
loaded_covariance_map = readmatrix('SAVED_DATA\map_landmark_covariance.txt');

loaded_landmark_vector = cell(size(loaded_data_map,1),1);
j = 1;
for i=1:size(loaded_data_map,1)
    loaded_landmark_vector{i}.x(1) = loaded_data_map(i,1);
    loaded_landmark_vector{i}.x(2) = loaded_data_map(i,2);
    loaded_landmark_vector{i}.P(1,1) = loaded_covariance_map(j);
    loaded_landmark_vector{i}.P(1,2) = loaded_covariance_map(j+1);
    loaded_landmark_vector{i}.P(2,1) = loaded_covariance_map(j+2);
    loaded_landmark_vector{i}.P(2,2) = loaded_covariance_map(j+3);
    j = j + 4;
end
%% Plot
figure(1)
for i=1:length(loaded_landmark_vector)
    plot(loaded_landmark_vector{i}.x(1),loaded_landmark_vector{i}.x(2),'*b');
    hold on
end
hold off
figure(2)
for i=1:length(loaded_landmark_vector)
    plot(i,norm(loaded_landmark_vector{i}.P),'*r');
    hold on
end

