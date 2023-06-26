import math
import os
import time

import carla
import numpy as np

import connectionCarla
import matplotlib.pyplot as plt
from sklearn.cluster import KMeans, DBSCAN
from PIL import Image


def get_connection(townName="Town03", host="localhost", reloadWorld=True,
                   camaraLocation=carla.Location(140, -102, 39.6), camRotation=None, render=True,
                   syncMode=False,
                   port=2000, delta=0.05):
    conncetion = connectionCarla.CarlaConnection()
    conncetion.__int__(townName=townName, host=host, reloadWorld=reloadWorld,
                       camaraLocation=camaraLocation, camRotation=camRotation, render=render, syncMode=syncMode,
                       port=port, delta=delta)
    return conncetion

def get_filtered_files(folder_path):
    files = os.listdir(folder_path)
    return [f for f in files if f[0].isdigit() and f[1] == "_"]


def groups_contain(row, groups):
    id_vehicle = row[0]
    id_walker = row[3]
    timeframe = row[7]
    count = 0
    for line in groups:
        if line[0] == id_vehicle and (-10 < line[7] - timeframe < 10):
            return True, count
        count += 1
    return False, -1


def load_data(path_prefix):
    data = []
    for i in range(20):
        file_path = path_prefix + str(i) + "_array_data.txt"
        if os.path.exists(file_path):
            with open(file_path, 'r') as file:
                file_data = file.readlines()
                for line in file_data:
                    line_data = eval(line.strip())
                    line_data.insert(9, i)
                    data.append(line_data)
    return data


def process_groups(data):
    groups = []
    for line in data:
        values = line
        in_array, count = groups_contain(values, groups)
        if in_array:
            if values[6] < groups[count][6]:
                groups[count] = values
        else:
            groups.append(values)
    return groups


def get_circle_sizes(ttc_values, min_size=100, max_size=10000):
    sizes = []
    for ttc in ttc_values:
        size = math.pow((1.5 - ttc) / 1.5, 4) * max_size
        sizes.append(size)
    return sizes


def show_scala_color():
    # Scatter plot sclala
    x_cords = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    y_cords = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
    ttc_s = [0, 0.15, 0.3, 0.45, 0.6, 0.75, 0.9, 1.05, 1.2, 1.35, 1.5]
    plt.scatter(x_cords, y_cords, c=ttc_s, cmap='inferno', alpha=1, s=100, edgecolors='none')
    for i, txt in enumerate(ttc_s):
        plt.text(x_cords[i] + 0.005, y_cords[i], "TTC = " + str(round(txt, 2)), ha='left', va='center', fontsize=12)
    plt.show()


def show_situation_in_Carla(data, conncetion=None):
    if conncetion is None:
        conncetion = get_connection()
    for x in data:
        index = x[9]
        start = max(x[7] - 10, 0)
        duration = 15
        vehicleID = x[0]
        print(x, filtered_files[index], int(start), duration, vehicleID)
        conncetion.client.replay_file("D:/" + filtered_files[index], int(start), duration, vehicleID)
        time.sleep(duration + 6)


def plot_points_Carla(x_cords, y_cords, ttc_s, cc=None):
    if cc is None:
        cc = get_connection()
    for x, y, i in zip(x_cords, y_cords, ttc_s):
        print(x, y, i)
        location = carla.Location(x, y, 5)
        ttc = i
        cc.draw_waypoint(location=location, index="x", life_time=500, intensity=1)


def kmean_plot(data, n_clusters=20):
    kmeans = KMeans(n_clusters=n_clusters)  # You can choose the number of clusters as needed
    kmeans.fit(data)
    labels = kmeans.labels_
    n_elements = np.bincount(labels)
    # # Print the results
    for i, count in enumerate(n_elements):
        print(f"Cluster {i+1}: {count} elements")
    plt.scatter(x_cords, y_cords, c=labels)
    plt.show()

    # Plot it as a bar chart
    # plt.bar(range(len(n_elements)), n_elements)
    # plt.xlabel("Cluster")
    # plt.ylabel("Number of elements")


def dbscanClustering(data):
    # Cluster the points using DBSCAN
    dbscan = DBSCAN(eps=20, min_samples=3)
    labels = dbscan.fit_predict(data)

    # Count the number of elements in each cluster
    n_elements, counts = np.unique(labels, return_counts=True)
    for i, count in zip(n_elements, counts):
        if i != -1:
            print(f"Cluster {i+1}: {count} elements")
        else:
            print(f"Noise: {count} elements")

    plt.scatter(x_cords, y_cords, c=labels, cmap='inferno', alpha=1, s=100, edgecolors='none')
    # Label each cluster with a number
    unique_labels = set(labels)
    for label in unique_labels:
        if label == -1:
            continue
        # Get the indices of data points with this label
        indices = np.where(labels == label)[0]
        # Calculate the centroid of the cluster
        centroid = np.mean(data[indices], axis=0)
        # Add a text label at the centroid
        plt.text(centroid[0], centroid[1], str(label + 1), color='white', fontsize=12, weight='bold', ha='center',
                 va='center')

    # Set axis labels and title
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("DBSCAN Clustering")
    plt.show()


# Define a function to get circle sizes based on ttc
def get_circle_sizes(ttc_values, min_size=100, max_size=10000):
    sizes = []
    for ttc in ttc_values:
        size = math.pow((1.5 - ttc) / 1.5, 4) * max_size
        sizes.append(size)
    return sizes


def get_circle_sizes2(ttc_values, speeds, ttc_min=0, ttc_max=1.5, speed_max=50, min_size=100, max_size=3000):
    sizes = []
    blub = []
    for ttc, speed in zip(ttc_values, speeds):
        # Normalize TTC and speed values to a range between 0 and 1
        norm_ttc = (ttc) / (ttc_max)
        norm_speed = speed / speed_max

        # Calculate the size based on normalized TTC and speed values
        size = min_size + (1 - norm_ttc) * norm_speed * (max_size - min_size)
        sizes.append(size)
        print(round(ttc,2), round(speed,2))
        blub.append([size, ttc, speed])

    print("---------")
    sorted_blub = sorted(blub, key=lambda x: x[1])
    for x in sorted_blub:
        print("blub",x)


    return sizes


def plot_cluster_dbscan_ax(ax, data, ttcs):
    # Convert ttcs to a NumPy array
    ttcs = np.array(ttcs)
    # Cluster the points using DBSCAN
    dbscan = DBSCAN(eps=18, min_samples=3)
    labels = dbscan.fit_predict(data)

    # Count the number of elements in each cluster
    n_elements, counts = np.unique(labels, return_counts=True)
    for i, count in zip(n_elements, counts):
        if i != -1:
            print(f"Cluster {i+1}: {count} elements")
        else:
            print(f"Noise: {count} elements")
    ax.scatter(x_cords, y_cords, c=labels, cmap='inferno', alpha=1, s=100, edgecolors='none')
    # Label each cluster with a number
    unique_labels = set(labels)
    for label in unique_labels:
        if label == -1:
            continue
        # Get the indices of data points with this label
        indices = np.where(labels == label)[0]
        # Calculate the centroid of the cluster
        centroid = np.mean(data[indices], axis=0)
        # Add a text label at the centroid
        ax.text(centroid[0], centroid[1], str(label + 1), color='white', fontsize=12, weight='bold', ha='center',
                va='center')

        # Find the smallest TTC in the cluster
        cluster_ttcs = ttcs[indices]
        min_ttc = np.min(cluster_ttcs)
        print(f"Cluster {label+1}: Smallest TTC = {round(min_ttc,2)}")


def plot_points_ax(ax, x_cords, y_cords, ttc_s, circle_size=100):
    ax.scatter(x_cords, y_cords, c=ttc_s, cmap='inferno', alpha=1, s=circle_size, edgecolors='none')


def plot_points_with_cirle_ax(ax, x_cords, y_cords, ttc_s, circle_sizes, innercircle_size=50, speeds=None):
    # Plot the points
    ax.scatter(x_cords, y_cords, c=ttc_s, cmap='inferno', alpha=0.3, s=circle_sizes, edgecolors='none')
    ax.scatter(x_cords, y_cords, c=ttc_s, cmap='inferno', alpha=1, s=innercircle_size, edgecolors='none')

    for (i, blub) in enumerate(zip(x_cords, y_cords, ttc_s, speeds)):
        x, y, ttc, speed = blub
        print((10-i+2), x, y, ttc, speed)
        if ttc == 0 or ttc == 1.5:
            continue
        ax.text(x+i*2, y, str(10-i+2), color='white', ha='center', va='center', fontsize=8)


folder_path = "D:/"

# Get all files in the folder
files = os.listdir(folder_path)

# Filter files that start with a number followed by an underscore
filtered_files = get_filtered_files(folder_path)

print(filtered_files)

# path_prefix = "D:/GIT/masterarbeit/tmp/ywalk03/"
path_prefix = "D:/GIT/masterarbeit/"
data = load_data(path_prefix)
groups = process_groups(data)
sorted_data = sorted(groups, key=lambda x: -x[6])

# Shows the scala color form 0 to 1.5
# show_scala_color()
x_cords = [-300, -280]
y_cords = [-300, -300]
ttc_s = [0, 1.5]
speeds = [0, 10]
# x_cords = []
# y_cords = []
# ttc_s = []
# speeds = []

for x in sorted_data[-10:]:
    # if x[6] < 0.5:
    x_cords.append(x[1])
    y_cords.append(x[2])
    ttc_s.append(x[6])
    speeds.append(x[8])
# plot_points_Carla(x_cords, y_cords, ttc_s)

# Show the situation in Carla
# conncetion = get_connection()
# sorted_data = sorted(groups, key=lambda x: x[6])
# eintrag = 2
# show_situation_in_Carla(sorted_data[-eintrag:-(eintrag-1)], conncetion)
# exit(0)

print("speeds:", min(speeds), max(speeds))
print("ttc:", min(ttc_s), max(ttc_s))
# Spiegeln der Punkte um 180 Grad um die horizontale Achse wegen Carla
y_cords = [-y for y in y_cords]

# Prepare the data for clustering
data = np.column_stack((x_cords, y_cords))

# Cluster the data using KMeans algorithm
# kmean_plot(data, n_clusters=3)

# Cluster the data using DBSCAN algorithm
# dbscanClustering(data)

# Load the background image
image_path = ".\carla_map_clean.PNG"
background_image = Image.open(image_path)

scale_factor_h = 0.415  # Ändern Sie diesen Wert, um die Skalierung anzupassen
scale_factor_w = 0.41  # Ändern Sie diesen Wert, um die Skalierung anzupassen
img_w, img_h = background_image.size
img_w, img_h = int(img_w * scale_factor_w), int(img_h * scale_factor_h)
background_image = background_image.resize((img_w, img_h), Image.LANCZOS)

# Calculate the extent values
center_x, center_y = 185 * scale_factor_w, 0
left, right = center_x - img_w / 2, center_x + img_w / 2
bottom, top = center_y - img_h / 2, center_y + img_h / 2

# Create a plot with the same aspect ratio as the background image
fig, ax = plt.subplots(figsize=(img_w / 80, img_h / 80))

# Display the background image
ax.imshow(background_image, extent=(left, right, bottom, top))

# Get circle sizes based on ttc and speed
circle_sizes = get_circle_sizes2(ttc_s, speeds, max_size=10000)

# plot_points_ax(ax, x_cords, y_cords, ttc_s, 50)
# plot_points_with_cirle_ax(ax, x_cords, y_cords, ttc_s, circle_sizes, innercircle_size=70, speeds=speeds)
plot_cluster_dbscan_ax(ax, data, ttc_s)


# Load the second image
second_image_path = ".\Scala3.PNG"
second_image = Image.open(second_image_path)

# Add the second image to the upper right corner
ax2 = fig.add_axes([0.68, 0.175, 0.2, 0.2], anchor='NE', zorder=1)
ax2.imshow(second_image)
ax2.axis('off')

# Remove axes and show the plot
ax.axis('off')
plt.show()
