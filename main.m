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


%% Ground truth comparison
if GT
    gt = readmatrix(select_GT);

    figure(10),clf;
    plot(gt(:,2),gt(:,3),'-k');
    hold on
    for i = 1:length(pos_robot)
        
        posx(i) = pos_robot{i,1}(1);
        posy(i) = pos_robot{i,1}(2);
        th(i) = pos_robot{i,1}(3);

        covx(i) = cov_robot{i,1}(1,1);
        covy(i) = cov_robot{i,1}(2,2);  
        covt(i) = cov_robot{i,1}(3,3);  
    end 

    plot(posx,posy,'-r')
    legend('Ground truth','Estimated','Location','best')
    title('GT Vs. EST')
    xlabel ('x [m]');
    ylabel ('y [m]');
    %
    figure(5),clf
    errx = gt(:,2)' - posx;
    plot(1:1:length(gt),errx)
    hold on
    erry = gt(:,3)' - posy;
    plot(1:1:length(gt),erry)
    legend('Est. error x','Est. error y','Location','best')
    title('Estimation errors x and y')
    xlabel ('Iteration');
    ylabel ('[m]');

    %
    figure(6), clf;
    plot(1:1:length(gt),errx)
    hold on
    plot(1:1:length(gt),2*covx,'r');
    hold on
    plot(1:1:length(gt),-2*covx,'r');
    hold on
    plot(1:1:length(gt),3*covx,'g');
    hold on
    plot(1:1:length(gt),-3*covx,'g');

    legend('Est. error x','Est. covariance x (97%)','','Est. covariance x (99%)','Location','best')
    title('Estimation errors x')
    xlabel ('Iteration');
    ylabel ('[m]');
    %
    figure(7), clf;
    plot(1:1:length(gt),erry)
    hold on
    plot(1:1:length(gt),2*covy,'r');
    hold on
    plot(1:1:length(gt),-2*covy,'r');
    hold on
    plot(1:1:length(gt),3*covy,'g');
    hold on
    plot(1:1:length(gt),-3*covy,'g');

    legend('Est. error y','Est. covariance y (97%)','','Est. covariance y (99%)','Location','best')
    title('Estimation errors y')
    xlabel ('Iteration');
    ylabel ('[m]');

    %
    figure(12), clf;

    errt = gt(:,4)' - th;
    plot(1:1:length(gt),errt);
    hold on
    plot(1:1:length(gt),2*covt,'r');
    hold on
    plot(1:1:length(gt),-2*covt,'r');
    hold on
    plot(1:1:length(gt),3*covt,'g');
    hold on
    plot(1:1:length(gt),-3*covt,'g');

    legend('Est. error theta','Est. covariance theta (97%)','','Est. covariance theta (99%)','Location','best')
    title('Estimation errors theta')
    xlabel ('Iteration');
    ylabel ('[rad]');
end

% Saving the data
if save_datas == true
    main_save_map
end