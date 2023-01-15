# Robotic Action and Perception
The current repository contains the code developed for the course [_Robotic Action and Perception_](https://www.miro.ing.unitn.it/category/robotic-perception-and-action/), Masted Degree in Mechatronics Engineering - Department of Industrial Engineering - University of Trento, held by professors De Cecco Mariolino, Luchetti Alessandro. The project is supervised also by Tavernini Matteo, CEO of the startup [_Robosense_](https://www.robosense.it/it/)

# Running the code
To run the SLAM algorithm is necessary to load a set of data and properly configure the config.m file selecting the correct path. Then the main.m can ben executed  

# Laser Rangefinder Navigation
## Data
Data inside the [Data](Data/) folder are obtained by means of the [gridmap navigation simulator](https://www.mrpt.org/list-of-mrpt-apps/application_gridmapnavsimul/) application, an open source software that simulates the mobile robot motion as well as it mesures (with uncertainties); in particular we can generate the following files:
- `simul_LASER_LASER_SIM.txt`: each row contains the measurements of the LIDAR at each time step; in the particular case, the scan has a field of view of 180° that's divided in 361 samples;
- `simul_LASER_LASER_SIM_times.txt`: each row contains the absolute acquisition time of the respective LIDAR scan in the previous file;
- `simul_ODO.txt`: each line contains the increment of the robot odometry, variables $x,y,\theta$;
- `simul_ODO_times.txt`: each line contains the absolute acquisition time of the respective odometry increment.

## Repository management
In the [Scripts](Scripts/) folder there are developed the main script parts that are then imported in the [main](main.m) file:
- [load_data](Scripts/load_data.m): read laserscan and odometry data and performs some basic preprocessing;
- [plot_raw_data](Scripts/plot_raw_data.m): given the cellarray of laserscans in `load_data`, it provides some plots to visualize the raw provided data as well as the extracted features;
- [EKF](Scripts/EKF.m): contains the main loop of the Ektended Kalman Filter


# Classes
Different classes are developed to get different levels of abstraction of informations. [Naming and ideas mainly obtained by this article (also cited in the references).](https://www.iri.upc.edu/people/jsola/JoanSola/objectes/curs_SLAM/SLAM2D/SLAM%20course.pdf)

Everything has been coded in a Object-Oriented Programming to separate the main function and improve code readibility.


### [Laserscan](Classes/Laserscan.m)
Handles information of a laserscan. It takes as input a the polar measurement and is able to convert them in cartesian space. 

Internally embedds an algorithm to extract feature from the cartesian map thus generating a vector of observations for the given data that will be used in the Kalman prediction step as well as in the map update.

### [Odometry](Classes/Odometry.m)
Stores information of the odometry system on the robot in order to have easier comput.


### [Robot](Classes/Robot.m)
Stores current information of the robot, in particular it's state estimate and covariance matrix. 

It contains function that allows for a simple calculation of jacobians used in the EKF.


### [Observation](Classes/Observation.m)
An observation is regarded as a feature extracted in the robot's reference frame and it's characterized by a cartesian position (w.r.t. the robot) and an uncertainty that's computed based on the one of the LiDAR.


### [Landmark](Classes/Landmark.m)
A landmark can be regarded as an observation projected into a fixed reference frame; it's described by a cartesian coordinate and it's uncertainty. 

The goal of the SLAM is thus to localize the robot based on the landmark that it sees and handling newly seen landmarks.


### [Map](Classes/Map.m)
Handles the structure of the map, storing all landmarks inside it and providing functions aiming at closing the loop and computing the correspondences between current observations and landmarks inside the map itself.


# Notes
File [`load_data.m`](load_data.m) reads the simulated [data](Data/) and builds cell-arrays of `Laserscan` and `Odometry` objectes. While loading the laserscans, it also pre-computes the features to reduce the overhead at EKF runtime. Since this is a highly time-consuming operation, the pre-processed cell-arrays are stored in a folder [ProcessedData](ProcessedData/) where they can be cleanly loaded each other time. This can be achieved by enabling the following flag in the [main](main.m) script:
```matlab
load_precomputed_data = true;
```
If no pre-processed data are present in the folder, still raw data are read and all features are extracted, saving the computation in the [ProcessedData](ProcessedData/) folder. If simulation [data](Data/) are replaced, we can so act in two way: by setting the flag `load_precomputed_data = false` or by simply deleting the files in [ProcessedData](ProcessedData/).



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
- [_Simulataneous localization and mapping with the extended Kalman filter - A very quick guide... with Matlab code!_](https://www.iri.upc.edu/people/jsola/JoanSola/objectes/curs_SLAM/SLAM2D/SLAM%20course.pdf)
- [_SLAM for Dummies_](http://dspace.mit.edu/bitstream/handle/1721.1/36832/16-412JSpring2004/NR/rdonlyres/Aeronautics-and-Astronautics/16-412JSpring2004/A3C5517F-C092-4554-AA43-232DC74609B3/0/1Aslam_blas_report.pdf)

References for building the map:
- [_Feature Selection Criteria for Real Time EKF-SLAM Algorithm_](https://journals.sagepub.com/doi/epub/10.5772/7237)
