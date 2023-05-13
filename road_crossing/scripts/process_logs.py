import matplotlib.pyplot as plt
import numpy as np
import sys
import shapely.geometry as geom


colors = ["blue", "red", "green", "magenta", "olive", "orange", "cyan"]

def plot_graph(data, labels, step = False, legend = None, title = None):
    if step:
        plt.step(data[0], data[1], where="post", linewidth=2)
        plt.xlim(0, np.ceil(data[0][-1]))
    else:
        max_time = 0
        for i in range(len(data[0])):
            if (len(data[0][i]) == 1):
                if (legend[i] != "Min distance"):
                    plt.scatter(data[0][i], data[1][i], color=colors[i])
                else:
                    plt.scatter(data[0][i], data[1][i], color="black")
                    plt.annotate(round(data[1][i][0], 5), xy=(data[0][i][0]+0.3, data[1][i][0]-0.15))
            else:
                plt.plot(data[0][i], data[1][i], linewidth=2, color=colors[i])
            max_time = max_time if max_time > max(data[0][i]) else max(data[0][i])
        plt.xlim(0, np.ceil(max_time))
    plt.xlabel(labels[0])
    plt.ylabel(labels[1])
    plt.grid(True)
    if legend is not None:
        plt.legend(legend)
    if title is not None:
        plt.title(title)
    plt.show()

def velocity_graph(file_name):
    times = []
    velocities = []
    try:
        with open(file_name, "r") as fp:
            lines = fp.readlines()
    except FileNotFoundError:
        print("File not found")
        exit()

    for line in lines:
        if "Movement started" in line:
            time = float(line[13:33])
            times.append(time)
            velocities.append(0.0)
            continue
        elif "Crossing finished" in line:
            time = float(line[13:33])
            times.append(time)
            velocities.append(0.0)
            break
        elif "velocity" not in line:
            continue
        else:
            time = float(line[13:33])
            velocity = float(line[-13:-5])
            times.append(time)
            velocities.append(velocity)

    times = np.array(times)
    times -= times[0]
    data = [times, velocities]
    labels = ["Time [s]", "Velocity [m/s]"]

    plot_graph(data, labels, True)

def dist_center_graph(file_name):
    times = []
    distances = []
    veh_ids = []
    min_dist = float("inf")
    min_dist_time = 0

    try:
        with open(file_name, "r") as fp:
            lines = fp.readlines()
    except FileNotFoundError:
        print("File not found")
        exit()

    for line in lines:
        if "Movement started" in line:
            time_start = float(line[13:33])
            continue
        elif "Crossing finished" in line:
            break
        elif "vehicle" not in line:
            continue
        else:
            time = float(line[13:33])
            line = line[61:-5]
            line = line.strip()
            num = line.split(",")
            veh_id = int(num[0])
            num.remove(num[0])
            num[0] = num[0][8:]
            num[1] = num[1][8:]
            dist = np.sqrt(np.sum(np.square(np.array([float(x) for x in num]))))
            if (min_dist > dist):
                min_dist = dist
                min_dist_time = time
            if (veh_id in veh_ids):
                distances[veh_ids.index(veh_id)].append(dist)
                times[veh_ids.index(veh_id)].append(time)
            else:
                veh_ids.append(veh_id)
                distances.append([dist])
                times.append([time])

    ret_times = []
    for t in times:
        t = np.array(t)
        t -= time_start
        ret_times.append(t)
    distances.append([min_dist])
    ret_times.append([min_dist_time - time_start])
    data = [ret_times, distances]
    labels = ["Time [s]", "Distance [m]"]
    legend = []
    for veh_id in veh_ids:
        legend.append("vehicle " + str(veh_id))
    legend.append("Min distance")
    if (len(legend) <= 1):
        legend = None

    plot_graph(data, labels, legend=legend)

def dist_graph(file_name):
    times = []
    distances = []
    veh_ids = []
    time_prev = []
    min_dist = float("inf")
    min_dist_time = 0

    robot_front = 1.1/2
    robot_back = -1.1/2
    robot_left = -0.5/2
    robot_right = 0.5/2
    coords = ((robot_front, robot_right), (robot_front, robot_left), (robot_back, robot_left), (robot_back, robot_right), (robot_front, robot_right))
    robot = geom.Polygon(coords)

    try:
        with open(file_name, "r") as fp:
            lines = fp.readlines()
    except FileNotFoundError:
        print("File not found")
        exit()

    for line in lines:
        if "Movement started" in line:
            time_start = float(line[13:33])
            continue
        elif "Crossing finished" in line:
            break
        elif "veh_pos" not in line:
            continue
        else:
            time = float(line[13:33])
            line = line[64:-5]
            line = line.strip()
            num = line.split(",")
            veh_id = int(num[0])
            if (veh_id in veh_ids and time - time_prev[veh_ids.index(veh_id)] < 0.2):
                continue
            num.remove(num[0])
            for i in range(len(num)):
                num[i] = float(num[i][5:])
            coords = ((num[0], num[2]), (num[0], num[3]), (num[1], num[3]), (num[1], num[2]), (num[0], num[2]))
            vehicle = geom.Polygon(coords)
            dist = robot.distance(vehicle)
            if (min_dist > dist):
                min_dist = dist
                min_dist_time = time
            if (veh_id in veh_ids):
                distances[veh_ids.index(veh_id)].append(dist)
                times[veh_ids.index(veh_id)].append(time)
                time_prev[veh_ids.index(veh_id)] = time
            else:
                veh_ids.append(veh_id)
                distances.append([dist])
                times.append([time])
                time_prev.append(time)

    ret_times = []
    for t in times:
        t = np.array(t)
        t -= time_start
        ret_times.append(t)
    distances.append([min_dist])
    ret_times.append([min_dist_time - time_start])
    data = [ret_times, distances]
    labels = ["Time [s]", "Distance [m]"]
    legend = []
    for veh_id in veh_ids:
        legend.append("vehicle " + str(veh_id))
    legend.append("Min distance")
    if (len(legend) <= 1):
        legend = None

    plot_graph(data, labels, legend=legend)

def time_to_contact_graph(file_name):
    pass

def main(args):
    file_name = args[1]
    velocity_graph(file_name)
    dist_center_graph(file_name)
    dist_graph(file_name)

if __name__ == '__main__':
    if (len(sys.argv) != 2):
        print("Usage process_logs.py <log_file>")
        exit()
    main(sys.argv)
