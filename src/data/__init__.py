import numpy as np
import math
import os

dataset = "Solomon/15/"

def readData(filename, truck_speed, drone_speed):

    path = os.getcwd() + '\data/TSPrd(time)/'

    with open(path+dataset+filename, "r") as file:
        lines = file.readlines()

    # Parse the data
    dimension = int(lines[0].split()[1])
    vehicle_capacity = int(lines[1].split()[1])
    number_of_vehicles = int(lines[2].split()[1])
    time_horizon = int(lines[3].split()[1])
    vertices = []
    for line in lines[5:]:
        data = line.split()
        if len(data) == 0:
            break
        xcoord, ycoord, demand, opening_tw, closing_tw, service_time, release_date = map(int, data)
        vertices.append((xcoord, ycoord, demand, opening_tw, closing_tw, service_time, release_date))

    vertices.append(vertices[0])
    n = len(vertices) - 2 

    w = np.zeros(n+2)
    t = np.zeros((n+2, n+2  ))
    d = np.zeros(n+2)

    for i in range(n+2):
       w[i] = vertices[i][6]

    for i in range(n+2):
        for j in range(n+2):
            t[i][j] = caculate_time(vertices[i], vertices[j], truck_speed)

    for i in range(n+2):
        d[i] = caculate_time_euclid(vertices[i], vertices[0], drone_speed)

    return n, w, t, d


def caculate_time(v1, v2, speed):
    distance = abs(v1[0] - v2[0]) + abs(v1[1] - v2[1])
    return distance / speed


def caculate_time_euclid(v1, v2, speed):
    distance = math.sqrt((v1[0] - v2[0])*(v1[0] - v2[0]) + (v1[1] - v2[1])*(v1[1] - v2[1]))
    return distance /speed


def getTruckRoute(x, solution, n):
    j = 0
    cur = 1
    pi = np.zeros(n+1)
    while cur < n+1:
        for i in range(n+2):
            if round(solution[x[j, i]]) == 1:
                j= i
                pi[cur] = i
                cur += 1
                break

    return pi
