# CarlaGlobalTTC
CarlaGlobalTTC is a comprehensive repository housing functions and methods for identifying
critical situations based on time-to-collision (TTC). The repository facilitates recording,
saving, and replaying these critical situations for further visualization and analysis.
To find critical situations can help to find dangerous situations in traffic and 
to improve the safety of autonomous vehicles.
## Installation and prequesites:
Please ensure the following software and libraries are installed:
- Install [Carla](https://carla.readthedocs.io/en/latest/start_quickstart/)
- Upgrade pip: ` pip3 install --upgrade pip`
- Install numpy: `pip3 install numpy`
- Install matplotlib: `pip3 install matplotlib`
- Install sklearn: `pip3 install sklearn`
- Install pandas: `pip3 install pandas`
- Install Shapely: `pip3 install shapely`

## Supported Versions
This repository has been tested with the following software versions:
- Python 3.8
- Carla 0.9.13
- pip 23.0.1
- numpy 1.23.5
- matplotlib 3.6.3
- sklearn 0.0.post1
- pandas 1.5.3
- Shapely 2.0.1

## How to use
1. Start Carla 
2. Record simulation with timeToCollision.py 
   1. Change the amount of cars and walkers in the file
   2. Set up other parameters (e.g. recording time, recording mode, ...)
3. Use different Tools to visualize the data
   1. DBSCANclusterData.py: Cluster the data with DBSCAN
   2. getCoordsFromRecord.py: Get the x/y-coordinates from the recorded data
   3. showFoundSituationsCarla.py: Show the x and y coordinates in Carla
   4. showPointsCarlaIntensity.py: Show the x and y coordinates in Carla with intensity
   5. visualizeData.py: Visualize the data with different plots and visualizations

## Files and functions
A brief description of the files and functions in this repository is provided below.
- barPlotDBSCAN.py: Visualizes the provided data (clusters, amount of situations, optimal TTC).
- connectionCarla.py: Manages the connection to Carla and controls the spawning of cars and walkers.
- DBSCANclusterData.py: Reads information and clusters them using DBSCAN.
- recordingModeCollisionsOnly.py: Records and saves all collisions within a simulation.
- showFoundSituationsCarla.py: Replays a provided critical situation.
- showPointsCarlaIntensity.py: Visualizes found critical points in Carla.
- timeToCollision.py: Main script for identifying and saving critical situations based on the TTC.
- visualizeData.py: Provides various visualizations of the saved data.


## Possible Extensions
- Use another metric then Time to Collision
- Add more visualization tools
- Validate the results with real world data
- Variate the parameters
- ...

