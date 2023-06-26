import matplotlib.pyplot as plt

# Data
import numpy as np

# Number of elements in each cluster
data = [7, 80, 98, 31, 15, 70, 171, 21, 13, 5, 6, 68, 16, 6, 42, 4, 6, 9, 29, 4, 3, 3]

# X-axis labels
labels = ['Noise', 'C: 1', 'C: 2', 'C: 3', 'C: 4', 'C: 5', 'C: 6', 'C: 7', 'C: 8', 'C: 9', 'C: 10', 'C: 11', 'C: 12', 'C: 13', 'C: 14', 'C: 15', 'C: 16', 'C: 17', 'C: 18', 'C: 19', 'C: 20', 'C: 21']

# TTC values for each cluster
ttc = [None, 0.51, 0.51, 0.95, 0.85, 0.51, 0.31, 0.31, 0.71, 0.71, 0.61, 0.31, 0.71, 0.85, 0.51, 1.35, 0.85, 1.25, 0.21, 1.15, 0.61, 0.31]

# Create a color map
norm = plt.Normalize(0, 1.5)
colors = plt.cm.inferno(norm(ttc[1:]))
colors = np.concatenate((np.array([[0, 0, 0, 1]]), colors), axis=0)
# Create a bar plot
plt.bar(labels, data, color=colors)

# Add x-axis and y-axis labels
plt.xlabel('Clusters', fontsize=22)
plt.ylabel('Elemente pro Cluster', fontsize=22)

# Add TTC labels rotated by 90 degrees
for i in range(len(labels)):
    if i != 0:
        plt.text(i, data[i]+2, f"TTC: {ttc[i]}", ha='center', rotation=90, fontsize=18)

# Set the plot title
# plt.titlse('Elements in Each Cluster', fontsize=20)

# Set the height of the chart
plt.ylim(0, 220)
plt.tick_params(axis='x', labelsize=14)
# Display the plot
plt.show()
