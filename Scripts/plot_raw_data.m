if pre_compute_features
    disp('Computing features for all scans...');
    for k = 1:length(laserscans)
        laserscans{k}.extract_feature();
    end
    disp('Done.');
end

if plot_animation
    
    fs_plot = 10;
    fs_meas = 1 / dt;
    i_step  = round(fs_meas / fs_plot);
    figure(4), clf, hold on;
    for i = 1:i_step:length(laserscans)

        clf;
        plot(laserscans{i})
        pause(dt);

    end

    clearvars fs_plot fs_meaas i_step i
end

clearvars plot_animation pre_compute_features

