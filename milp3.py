import time
import numpy as np
import math
import sys
from docplex.mp.model import Model
from src.data import readData

time_limit = 1000 * 60 * 30
truck_speed = 30/60
drone_speed = 60/60
A = 4
delta = 5

M = 100000
enduration = 90

n, w, t, d = readData("C101_1.5.dat", truck_speed, drone_speed)

_k = 2 #Number of truck
_c = 1 #Number of drone

K = [i for i in range(1, _k + 1)]
C = [i for i in range(1, _c + 1)]


a = [1 for i in range(n+2)]
a[0] = 0
a[n+1] = 0

N = [i for i in range(1, n+1)]
N_0 = [i for i in range(n+1)]
N_n1 = [i for i in range(1, n+2)]
N_0n1 = [i for i in range(n+2)]

D = [i for i in range(1, n+1) if d[i] <= enduration]
D_0 = [i for i in range(n+1) if d[i] <= enduration]
D_n1 = [i for i in range(1, n+2) if d[i] <= enduration]





mdl = Model('integer_programming')

#Var
x = {} # 1, if node j is visited immediately after node i by the truck. 0, otherwise.
r = {} # 1, if node j is visited (not necessarily immediately) after node i by the drone. 0
u = {} # 1, if the drone flies to node i to resupply the truck with new orders. 0, otherwise.
y = {} # 1, if the order of customer j is loaded onto the truck at node i. 0, otherwise.
T = {} #Time when the truck departs node i.
s = {} #Time when the drone is launched for node i.
e = {} #Elapsed time between arrival and departure of the truck at node i.
index  = {}

#1
for k in K:
    for i in N_0n1:
        for j in N_0n1:
            x[k, i, j] = mdl.binary_var(name=f'x_{k}/{i}_{j}')

#
for c in C:
    for i in D_0:
        for j in D_n1:
            if j == i: continue
            r[(c, i, j)] = mdl.binary_var(name=f'r_{c}/{i}_{j}')

#
for c in C:
    for i in D_0:   
        u[c , i] = mdl.binary_var(name=f'u_{c}/{i}')

#
for c in C:
    for i in D_0:
        for j in N:
            y[c, i, j] = mdl.binary_var(name=f'y_{c}/{i}_{j}')

#
for k in K:
    for i in N_0n1:
        T[k, i] = mdl.continuous_var(name=f'T_{k}/{i}')


for k in K:
    for i in N_0n1:
        index[k, i] = mdl.integer_var(name=f'index_{k}/{i}')

maxT = mdl.continuous_var(name=f'maxT')

#
for c in C:
    for i in D:
        s[c, i] = mdl.continuous_var(name=f's_{c}/{i}')

for k in K:
    for i in D:
        e[k, i] = mdl.continuous_var(name=f'e_{k}/{i}')

for k in K:
    for i in N:
        mdl.add_constraint(x[k, i, i] == 0)

#Constraint

#2, Router of all truck must start at deport
for k in K:
    mdl.add_constraint(mdl.sum(x[k, 0, j] for j in N) == 1)

#3, Router of all the truct must end at deport
for k in K:
    mdl.add_constraint(mdl.sum(x[k, i, n+1] for i in N) == 1)

#4, All the node will have just 1 time come
for j in N:
    mdl.add_constraint(mdl.sum(x[k, i, j] for k in K for i in N_0 if j != i) == 1)

#5, All the node will have just 1 time leave
for i in N:
    mdl.add_constraint(mdl.sum(x[k, i, j]  for k in K for j in N_n1 if j != i) == 1)

#5.1, The node must be visit and leave by the same truck
for k in K:
    for i in N:
        mdl.add_constraint(mdl.sum(x[k, i, j] for j in N_n1 if j != i) == mdl.sum(x[k, j, i] for j in N_0 if i != j))

#5.2 for the truck routing
if (_k != 1):
    for k in K:
        for i in N_0n1:
            for j in N_0n1:
                if (i != j):
                    mdl.add_constraint(index[k, j] - index[k, i] >= 1 - (n+1) * (1 - x[k, i , j]))


    
#Routing for drone

# #6
for c in C:
    for i in D:
        mdl.add_constraint(u[c, i] <= u[c, 0])

mdl.add_constraint(sum(u[c, i] for c in C for i in D_0) >=1)

# #7
for c in C: 
    mdl.add_constraint(mdl.sum(r[c, 0 , j] for j in D) == u[c, 0])

# #8
for c in C: 
    mdl.add_constraint(mdl.sum(r[c, i , n+1] for i in D) == u[c, 0])

# #9
for c in C:
    for j in D:
        mdl.add_constraint(mdl.sum(r[c, i , j] for i in D_0 if i != j) == u[c, j])

# # #10
for c in C:
    for i in D:
        mdl.add_constraint(mdl.sum(r[c, i , j] for j in D_n1 if i != j) == u[c, i])


# # #10.1
for i in D:
    mdl.add_constraint(mdl.sum(u[c, i] for c in C) <= 1)


# # # # 11 Drone capacity
for c in C:
    for i in D:
        mdl.add_constraint(mdl.sum(y[c, i, j] * a[j] for j in N) <= A * u[c, i])

# # #12 
for j in N:
    mdl.add_constraint(mdl.sum(y[c, i, j] for c in C for i in D_0) == 1)

for c in C:
    for i in N:
        for j in N:
            for k in K:
                mdl.if_then(y[c, i, j] == 1, sum(x[k, l, i] for l in N_0) + sum(x[k, h, j] for h in N_0) == 2)

#13
for k in K:
    for j in N:
        mdl.add_constraint(T[k, j] >= w[j] + min(d[j], t[0, j]))

# # #14
for k in K:
    for i in D_0:
        for j in N :
            if j == i:continue
            mdl.add_constraint(T[k, j] >= T[k, i] - M * (1 - mdl.sum(y[c, i, j] for c in C)))

# # # # #15
for k in K: 
    for i in N_0:
        for j in N_n1:
            if ((j in D) or (j == i)): continue
            mdl.add_constraint(T[k, j] >= T[k, i] + t[i, j] - M * (1 - x[k, i, j]))

# # #16
for k in K:
    for i in N_0:
        for j in D :
            if j == i:continue
            mdl.add_constraint(T[k, j] >= T[k, i] + t[i,j] + delta * sum(u[c, j] for c in C) - M * (1 - x[k, i, j]))

# # #17
# for k in K:
    for c in C:
        for j in D:
            mdl.add_constraint(T[k, j] >= s[c, j] + d[j] + delta - M*(1 - u[c, j]))


# #18
for c in C:
    for k in K:
        for i in D:
            for j in D:
                if i == j: continue
                mdl.add_constraint(s[c, j] >= T[k, i] + d[i] - M * (1 - r[c, i, j]))


# # #19
for c in C:
    for i in N:
        for j in D:
            mdl.add_constraint(s[c, j] >= w[i]*y[c,j,i])


#20
for k in K:
    for j in N:
        mdl.add_constraint(T[k, 0] >= w[j] * mdl.sum(y[c, 0, j] for c in C) - M * (1 - mdl.sum(x[k, i, j] for i in N_0)))

# #21
for k in K:
    for j in D: 
        for i in N_0:
            if i == j:continue
            mdl.add_constraint(e[k, j] >= T[k, j] - (T[k, i] + t[i,j]) - M * (1 - x[k, i, j]))

# #22
for k in K:
    for j in D:
        mdl.add_constraint(e[k, j] <= M * mdl.sum(u[c, j] for c in C))


# #23
for k in K:
    for i in N:
        mdl.add_constraint(T[k, n+1] >= T[k, i] + t[i,n+1])

# #24
for k in K:
    mdl.add_constraint(T[k, n+1] >= T[k, 0] + mdl.sum(t[i,j] * x[k, i, j] for i in N_0n1 for j in N_0n1) + mdl.sum(e[k, i] for i in D))

#The optiomal need
for k in K:
    mdl.add_constraint(maxT >= T[k, n+1])
mdl.minimize(maxT)

start_time = time.time()
solution = mdl.solve()
end_time = time.time()
solve_time = end_time - start_time
print('Solve Time:', solve_time)
print(solution)




