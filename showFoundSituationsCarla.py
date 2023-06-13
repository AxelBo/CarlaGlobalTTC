
import carla
import connectionCarla
import pandas as pd

# Load the data from the file into a pandas DataFrame
data = pd.read_csv('recordedSituations.txt', header=None)

# Extract the x and y coordinates for the vehicles
x = data[1].astype(float)
y = data[2].astype(float)


conncetion = connectionCarla.CarlaConnection()
conncetion.__int__(reloadWorld=True)

for i,j in zip(x,y):
    location = carla.Location(x=i, y=j, z=10)
    conncetion.draw_waypoint(location, "x", 1000)
exit()