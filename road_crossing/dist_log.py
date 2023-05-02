import numpy as np
import subprocess


'''
Calculate the largest and smallest distance between the robot and the vehicle
'''
def main():
    cmd_str = "tr '\r' '\n' < log.log | grep -F 'vehicle' > new.log"
    subprocess.run(cmd_str, shell=True)

    max_dist = 0.0
    min_dist = float('inf')

    with open("new.log", "r") as fp:
        lines = fp.readlines()
    for line in lines:
        line = line[67:-5]
        line = line.strip()
        num = line.split(",")
        num[1] = num[1][8:]
        dist = np.sqrt(np.sum(np.square(np.array([float(x) for x in num]))))
        if dist > max_dist:
            max_dist = dist
        if dist < min_dist:
            min_dist = dist

    print("max_dist: {}".format(max_dist))
    print("min_dist: {}".format(min_dist))


if __name__ == "__main__":
    main()
