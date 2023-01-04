if plot_animation
    
    fs_plot = 10;
    fs_meas = 1 / dt;
    i_step  = round(fs_meas / fs_plot);
    figure(4), clf, hold on;
    for i = 1:i_step:length(laserscans)
        clf;
        plot(laserscans{i})
        title(['Time: ', num2str(laserscans_times(i)), 's']);
        pause(dt);
    end

    
end
clearvars fs_plot fs_meas i_step i

clearvars plot_animation

