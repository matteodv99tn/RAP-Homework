%% Initialization2  
% close all;
clear;  
clc;

addpath('Data')
addpath('Scripts')
addpath('Functions')
addpath('Classes')

% Load the configuration parameters for the algorithms
config
     
% Load the data
load_precomputed_data = true;
load_data


% Plot the raw data
plot_animation = false; % flat used to show (or not) an animation of the pointcloud
plot_raw_data


% Ground truth
GT = true;
if GT
    gt = readmatrix('simul_GT_1.txt');
end

EKF

%%
figure(3),clf;
plot(gt(:,2),gt(:,3),'-k');
hold on
for i = 1:length(pos_robot)
    posx(i) = pos_robot{i,1}(1);
    posy(i) = pos_robot{i,1}(2);
    covx(i) = cov_robot{i,1}(1,1);
    covy(i) = cov_robot{i,1}(2,2);

end

plot(posx,posy,'-r')
%%
figure(4)
errx = gt(:,2)' - posx;
plot(1:1:length(gt),errx)
hold on
erry = gt(:,3)' - posy;
plot(1:1:length(gt),erry)
%%
figure(5), clf;
plot(1:1:length(gt),errx)
hold on
plot(1:1:length(gt),2*covx,'r');
hold on
plot(1:1:length(gt),-2*covx,'r');
%% 
figure(6), clf;
plot(1:1:length(gt),erry)
hold on
plot(1:1:length(gt),2*covy,'r');
hold on
plot(1:1:length(gt),-2*covy,'r');
