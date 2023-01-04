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
File [`load_data.m`](load_data.m) reads the [data](Data/) and converts all information into suitable structures for further manipulation, in particular:

- `laserscans` is a cell-array of `N_laserscans` object `Laserscan`; while loading, features are already pre-computed.
- `odometries` is a cell-array containing (relative) information of the odometries. Each cell is a struct containing:
    - `x` increment along the _forward direction_;
    - `y` increment along the _lateral direction_;
    - `theta` increment of the robot bearing;
- `laserscans_times` and `odometries_times` are the time vectors of the recorded laserscans and odometries respectively. Their sampling period is `dt_laserscans` and `dt_odometries` (`dt` is the mean sampling period).

**Note:** the extraction of the feature is time consuming, so once files are processed from the script, relevant information are stored in `.mat` files inside the folder `ProcessedData`. Before calling the `load_data.m` script, make sure that 
```matlab
load_precomputed_data = true;
```
is defined. If the flag is set to true, then it firstly tries to load (if available) data from the `ProcessedData` folder, otherwise it computes them from scratch. If the flag is set to false, the algorithm will always re-compute everything at each run.

## Classes
Avaliable classes:

- [Laserscan](Classes/Laserscan.m): handle everything about the laserscan; in particular it stores the polar information of the scan and translates them in cartesian coordinates. It embeds also a function that allows to extract features from the laserscan.

## Bibliography
References for feature extraction:
- [_A line segment extraction algorithm using laser data based on seeded region growing_](https://journals.sagepub.com/doi/pdf/10.1177/1729881418755245)
- [_Feature Selection Criteria for Real Time EKF-SLAM Algorithm_](https://journals.sagepub.com/doi/full/10.5772/7237#alg3-7237)

References for closure loop:
- [_A fast, complete, point cloud based loop closure for LiDAR odometry and mapping_](https://arxiv.org/pdf/1909.11811.pdf)
- [_Real-Time Loop Closure in 2D LIDAR SLAM_](https://static.googleusercontent.com/media/research.google.com/en//pubs/archive/45466.pdf)

References for EKF:
- [_A simple and efficient implementation of EKF - based SLAM relying on laser scanner in complex indoor environment_](https://www.infona.pl/resource/bwmeta1.element.baztech-5cbd9e8d-e5b6-4200-8f35-186220453ec8/content/partContents/0331eb36-015d-38a5-a06b-9f1870722f01)
- [_Slam with EKF, matlab code -> jaijuneja_](https://github.com/jaijuneja/ekf-slam-matlab)
- [_Slam with EKF, matlab code -> fatmakhalil_](https://github.com/fatmakhalil/EKF-SLAM-from-LiDAR-input-data)
