import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
import pandas as pd

# Load the data from the file into a pandas DataFrame
data = pd.read_csv('recordedSituations.txt', header=None)

# Extract the x and y coordinates for the vehicles
x = data[1].astype(float)
y = data[2].astype(float)

x_data = x
y_data = y
# plt.scatter(x, y)
# plt.show()



# Extract the ttc values
intensity = data[6].astype(float)
print(intensity.min())
for i in intensity:
    if i < 1.5:
        i = (1.5-i)/1.5
    else:
        i = 0
# Cluster the points using DBSCAN
dbscan = DBSCAN(eps=10, min_samples=5)
labels = dbscan.fit_predict(np.column_stack((x, y)))

# Plot the points and the circles
plt.scatter(x, y, c=labels, cmap='viridis')
for label in set(labels):
    if label == -1:
        continue
    x_cluster = x[labels == label]
    y_cluster = y[labels == label]
    intensity_cluster = intensity[labels == label]
    plt.scatter(x_cluster.mean(), y_cluster.mean(), s=pow(intensity_cluster.max(), 3) * 20,
                c='red', alpha=0.2)
plt.show()
exit()