# Robotic Action and Perception
The current repository contains the code developed for the course [_Robotic Action and Perception_](https://www.miro.ing.unitn.it/category/robotic-perception-and-action/), Masted Degree in Mechatronics Engineering - Department of Industrial Engineering - University of Trento, held by professors De Cecco Mariolino, Luchetti Alessandro. The project is supervised also by Tavernini Matteo, CEO of the startup [_Robosense_](https://www.robosense.it/it/)


# Laser Rangefinder Navigation


## Data
Data inside the [Data](Data/) folder are obtained by means of the [gridmap navigation simulator](https://www.mrpt.org/list-of-mrpt-apps/application_gridmapnavsimul/) application, an open source software that simulates the mobile robot motion as well as it mesures (with uncertainties); in particular we can generate the following files:
- `simul_LASER_LASER_SIM.txt`: each row contains the measurements of the LIDAR at each time step; in the particular case, the scan has a field of view of 180° that's divided in 361 samples;
- `simul_LASER_LASER_SIM_times.txt`: each row contains the absolute acquisition time of the respective LIDAR scan in the previous file;
- `simul_ODO.txt`: each line contains the increment of the robot odometry, variables $x,y,\theta$;
- `simul_ODO_times.txt`: each line contains the absolute acquisition time of the respective odometry increment.

## Repository management
In the [Scripts](Scripts/) folder there are developed the main script parts that are then imported in the [main](main.m) file:
- [load_data](Scripts/load_data.m): read laserscan and odometry data. It also process informations and embeds them in a single cell-array (see later);
- [plot_raw_data](Scripts/plot_raw_data.m): given the cellarray of laserscans in `load_data`, it provides some plots to visualize the raw provided data;


## Data structures
In the file `load_data.m` all data are aggregated in a vector `laserscans` of matlab's cell-arrays. In particular each entry present the following properties:
- `r`: a vector containing the 361 measurements of the LIDAR's measurement in polar coordinate; it's given the LIDAR span of $180^\circ$;
- `xscan, yscan`: projection in cartesian coordinates of the pointcloud. In this case it is assumed that the heading of the robot is coincident with the $x$ axis;
- `t_odo, t_lid`: the true times at which the odometry and lidar measurements are taken respectively;
- `t`: a newly define time based on the average sampling time; the first measurement starts at `t=1`;
- `x, y, theta`: the current absolute values of the robot provided by the odometry.


## Bibliography
References for feature extraction:
- [_A line segment extraction algorithm using laser data based on seeded region growing_](https://journals.sagepub.com/doi/pdf/10.1177/1729881418755245)
- [_Feature Selection Criteria for Real Time EKF-SLAM Algorithm_](https://journals.sagepub.com/doi/full/10.5772/7237#alg3-7237)

References for closure loop:
- [_A fast, complete, point cloud based loop closure for LiDAR odometry and mapping_](https://arxiv.org/pdf/1909.11811.pdf)
- [_Real-Time Loop Closure in 2D LIDAR SLAM_](https://static.googleusercontent.com/media/research.google.com/en//pubs/archive/45466.pdf)

References for EKF:
- [_A simple and efficient implementation of EKF - based SLAM relying on laser scanner in complex indoor environment_](https://www.infona.pl/resource/bwmeta1.element.baztech-5cbd9e8d-e5b6-4200-8f35-186220453ec8/content/partContents/0331eb36-015d-38a5-a06b-9f1870722f01)
