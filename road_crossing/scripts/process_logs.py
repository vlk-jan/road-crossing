#!/usr/bn/env python
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import sys
import shapely.geometry as geom


colors = ["blue", "red", "green", "magenta", "olive", "orange", "cyan"]

def plot_graph(data, labels, step = False, legend = None, title = None, save_name = None):
    max_time = 0
    if step:
        for i in range(len(data[0])):
            plt.step(data[0][i], data[1][i], color=colors[i], where="post", linewidth=2)
            max_time = max_time if max_time > max(data[0][i]) else max(data[0][i])
        plt.xlim(0, np.ceil(max_time))
    else:
        max_len = max(len(x) for x in data[0])
        for i in range(len(data[0])):
            if (len(data[0][i]) < max_len-1):
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
    #plt.ylim(0, 40)
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
    times_veh = []
    velocities_veh = []
    veh_ids = []
    try:
        with open(file_name, "r") as fp:
            lines = fp.readlines()
    except FileNotFoundError:
        print("File not found")
        exit()

    for line in lines:
        if "Movement started" in line:
            time_start = float(line[13:33])
            times.append(time_start)
            velocities.append(0.0)
            continue
        elif "Crossing finished" in line:
            time = float(line[13:33])
            times.append(time)
            velocities.append(0.0)
            break
        elif "velocity" not in line and "y_dot" not in line:
            continue
        else:
            time = float(line[13:33])
            velocity = float(line[-14:-5])
            if "y_dot" in line:
                veh_id = int(line[60:63])
                if (veh_id in veh_ids):
                    velocities_veh[veh_ids.index(veh_id)].append(velocity)
                    times_veh[veh_ids.index(veh_id)].append(time)
                else:
                    veh_ids.append(veh_id)
                    velocities_veh.append([velocity])
                    times_veh.append([time])
            else:
                times.append(time)
                velocities.append(velocity)

    ret_times = []
    times_veh.append(times)
    for t in times_veh:
        t = np.array(t)
        t -= time_start
        ret_times.append(t)
    velocities_veh.append(velocities)
    data = [ret_times, velocities_veh]
    save_name = file_name[:-4] + "_vel.pdf"
    labels = ["Time [s]", "Velocity [m/s]"]
    legend = []
    for veh_id in veh_ids:
        legend.append("vehicle " + str(veh_id))
    legend.append("robot")

    plot_graph(data, labels, True, save_name=save_name, legend=legend)

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

def time_to_contact(file_name):
    distances = []
    velocities = []
    times = []

    robot_front = 1.1/2

    try:
        with open(file_name, "r") as fp:
            lines = fp.readlines()
    except FileNotFoundError:
        print("File not found")
        exit()
    
    for i in range(len(lines)):
        if "Movement started" in lines[i]:
            time_start = float(lines[i][13:33])
            continue
        elif "Crossing finished" in lines[i]:
            break
        elif "velocity" in lines[i]:
            time = float(lines[i][13:33])
            vel = float(lines[i][-14:-5])
            velocities.append((time - time_start, vel))
        elif "veh_pos" not in lines[i]:
            continue
        else:
            time = float(lines[i][13:33])
            line = lines[i][64:-5]
            line = line.strip()
            num = line.split(",")
            num.remove(num[0])
            for j in range(len(num)):
                num[j] = float(num[j][5:])
            if (num[2] <= 0 and num[3] >= 0 or num[2] >= 0 and num[3] <= 0):
                dist = num[0] - robot_front
                if dist < 0:
                    continue
                distances.append((time - time_start, dist))
                
    for i in range(len(distances)):
        for j in range(len(velocities)):
            if (abs(distances[i][0] - velocities[j][0]) > 0.1):
                continue
            elif (velocities[j][1] == 0):
                continue
            time = distances[i][1]/velocities[j][1]
            times.append(time)
            break

    if (len(times) > 0):
        print("Average time to contact: " + str(sum(times)/len(times)) + " s")
        print("Minimal time to contact: " + str(min(times)) + " s")
    else:
        print("NA")

def time_till_dist(file_name, distance):
    times = []
    velocities = []
    dist = 0
    try:
        with open(file_name, "r") as fp:
            lines = fp.readlines()
    except FileNotFoundError:
        print("File not found")
        exit()

    for line in lines:
        if "Movement started" in line:
            time_start = float(line[13:33])
            times.append(0)
            velocities.append(0.0)
            continue
        elif "Crossing finished" in line:
            time = float(line[13:33]) - time_start
            times.append(time)
            velocities.append(0.0)
            break
        elif "velocity" not in line and "y_dot" not in line:
            continue
        else:
            time = float(line[13:33]) - time_start
            velocity = float(line[-14:-5])
            times.append(time)
            velocities.append(velocity)

    for i in range(len(times)-1):
        dist += velocities[i]*(times[i+1] - times[i])
        if (dist >= distance):
            return times[i+1]

def main(args):
    file_name = args[1]
    velocity_graph(file_name)
    dist_center_graph(file_name)
    dist_graph(file_name)
    animation_graph(file_name)
    time_to_contact(file_name)
    #print(time_till_dist(file_name, 2.69))

if __name__ == '__main__':
    
    if (len(sys.argv) != 2):
        print("Usage process_logs.py <log_file>")
        exit()
    main(sys.argv)
