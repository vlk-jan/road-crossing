#!/usr/bn/env python
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import sys
import shapely.geometry as geom


colors = ["blue", "red", "green", "magenta", "olive", "orange", "cyan"]

def plot_graph(data, labels, step = False, legend = None, title = None, save_name = None):
    if step:
        plt.step(data[0], data[1], color=colors[0], where="post", linewidth=2)
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
    plt.savefig(fname=save_name, format='pdf')
    plt.close()
    #plt.show()

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
    save_name = file_name[:-4] + "_vel.pdf"
    labels = ["Time [s]", "Velocity [m/s]"]

    plot_graph(data, labels, True, save_name=save_name)

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
    save_name = file_name[:-4] + "_center.pdf"

    plot_graph(data, labels, legend=legend, save_name=save_name)

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
    save_name = file_name[:-4] + "_dist.pdf"

    plot_graph(data, labels, legend=legend, save_name=save_name)

def get_positions(file_name):
    veh_pos = []
    rob_pos = []
    veh_ids = []

    max_x = 0
    max_y = 0
    min_x = float("inf")
    min_y = float("inf")

    try:
        with open(file_name, "r") as fp:
            lines = fp.readlines()
    except FileNotFoundError:
        print("File not found")
        exit()
    
    for line in lines:
        if "Movement started" in line:
            time_prev = float(line[13:33])
            continue
        elif "Crossing finished" in line:
            break
        elif "coords" not in line:
            continue
        else:
            time = float(line[13:33])
            line = line[61:-5]
            line = line.strip()
            num = line.split(",")
            veh_id = int(num[0])
            if (veh_id == 0 and time - time_prev < 0.15 and not len(rob_pos) == 0):
                continue
            elif (veh_id == 0):
                time_prev = time
            if (len(rob_pos) == 0 and veh_id != 0):
                continue
            num.remove(num[0])
            if (veh_id == 0):
                rob_pos.append((float(num[0]), float(num[1])))
            elif (veh_id in veh_ids):
                veh_pos[veh_ids.index(veh_id)].append((float(num[0]), float(num[1])))
            else:
                veh_ids.append(veh_id)
                veh_pos.append([(float(num[0]), float(num[1]))])
            max_x = max(max_x, float(num[0]))
            max_y = max(max_y, float(num[1]))
            min_x = min(min_x, float(num[0]))
            min_y = min(min_y, float(num[1]))

    if (len(veh_pos) > 1 and len(veh_pos[0]) > len(rob_pos)):
        veh_pos[0] = veh_pos[0][1:]
    return [[veh_pos, rob_pos], [min_x, max_x, min_y, max_y], veh_ids]

def animation_graph(file_name):
    data = get_positions(file_name)

    fig = plt.figure()
    plt.xlim(data[1][0], data[1][1])
    plt.xlabel("Easting [m]")
    plt.ylim(data[1][2], data[1][3])
    plt.ylabel("Northing [m]")
    veh_id = data[2]
    data = data[0]

    i = 0
    while i < len(data[0]):
        if (len(data[0][i]) != len(data[1])):
            data[0] = data[0][:i] + data[0][i+1:]
            veh_id = veh_id[:i] + veh_id[i+1:]
            i -= 1
        i += 1

    plt.gca().set_prop_cycle(color=colors[:len(data[0]) + 1])

    legend = ["robot"]
    for id in veh_id:
        legend.append("vehicle " + str(id)) 

    x = []
    y = []

    def animate(i):
        while True:
            app_x = [data[1][i][0]]
            app_y = [data[1][i][1]]
            for j in range(len(data[0])):
                app_x.append(data[0][j][i][0])
                app_y.append(data[0][j][i][1])

            x.append(app_x)
            y.append(app_y)
        
            plt.legend(legend)
            return plt.plot(x, y, marker='x')
    
    anim = animation.FuncAnimation(fig, animate, len(data[1]), repeat=True, repeat_delay=500, blit=True)
    #plt.show()
    anim.save((file_name[:-4] + '_traj.gif'), writer='imagemagick')

def time_to_contact_graph(file_name):
    pass

def main(args):
    file_name = args[1]
    velocity_graph(file_name)
    dist_center_graph(file_name)
    dist_graph(file_name)
    animation_graph(file_name)

if __name__ == '__main__':
    
    if (len(sys.argv) != 2):
        print("Usage process_logs.py <log_file>")
        exit()
    main(sys.argv)
