%% Initialization
close all;
clear;  
clc;

addpath('Data')
addpath('Scripts')
addpath('Functions')
addpath('Classes')

% Load the configuration parameters for the algorithms
config

% Load the data
load_data

ls  = laserscans{1};                                % choose a laserscan 
obj = Laserscan(ls.r, (ls.Theta) .* 3.14 / 180);    % build the object

extracted_features = obj.extract_feature();         % extract the features
% obj.features = extracted_features;

figure(1), clf, hold on;
plot(obj)
for k = 1:size(extracted_features, 1)               % plot the features

    i = extracted_features(k, 1);
    j = extracted_features(k, 2);
    line = extracted_features(k, 3:end);

    P1 = predict_point(line, obj.polar(i, 2));
    P2 = predict_point(line, obj.polar(j, 2));
    plot([P1(1), P2(1)], [P1(2), P2(2)], 'o-b', 'LineWidth', 1.5);
end
