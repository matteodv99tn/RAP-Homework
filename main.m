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
load_data

% Plot the raw data
plot_raw_data

% Run EKF
EKF


% Ground truth comparison
if GT
    gt = readmatrix(select_GT);

    figure(4),clf;
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
    figure(5),clf
    errx = gt(:,2)' - posx;
    plot(1:1:length(gt),errx)
    hold on
    erry = gt(:,3)' - posy;
    plot(1:1:length(gt),erry)
    %%
    figure(6), clf;
    plot(1:1:length(gt),errx)
    hold on
    plot(1:1:length(gt),2*covx,'r');
    hold on
    plot(1:1:length(gt),-2*covx,'r');
    %% 
    figure(7), clf;
    plot(1:1:length(gt),erry)
    hold on
    plot(1:1:length(gt),2*covy,'r');
    hold on
    plot(1:1:length(gt),-2*covy,'r');
end