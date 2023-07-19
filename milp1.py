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

n, w, t, d = readData("C101_1.dat", truck_speed, drone_speed)

a = [1 for i in range(n+2)]
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

#25-30
for i in N_0n1:
    for j in N_0n1:
        x[(i, j)] = mdl.binary_var(name=f'x_{i}_{j}')

for i in D_0:
    for j in D_n1:
        if j == i: continue
        r[(i, j)] = mdl.binary_var(name=f'r_{i}_{j}')

for i in D_0:
    u[i] = mdl.binary_var(name=f'u_{i}')

for i in D_0:
    for j in N:
        y[(i, j)] = mdl.binary_var(name=f'y_{i}_{j}')

for i in N_0n1:
    T[i] = mdl.continuous_var(name=f'T_{i}')

for i in D:
    s[i] = mdl.continuous_var(name=f's_{i}')
    e[i] = mdl.continuous_var(name=f'e_{i}')

#2
mdl.add_constraint(mdl.sum(x[(0, j)] for j in N) == 1)
#3
mdl.add_constraint(mdl.sum(x[(i, n+1)] for i in N) == 1)
#4
for j in N:
    mdl.add_constraint(mdl.sum(x[(i, j)] for i in N_0 if j != i) == 1)
#5
for i in N:
    mdl.add_constraint(mdl.sum(x[(i, j)] for j in N_n1 if i != j) == 1)



#6
for i in D:
    mdl.add_constraint(u[i] <= u[0])

#7
mdl.add_constraint(mdl.sum(r[0,j] for j in D) == u[0])

#8
mdl.add_constraint(mdl.sum(r[i,n+1] for i in D) == u[0])

# #9
for j in D:
    mdl.add_constraint(mdl.sum(r[i,j] for i in D_0 if i != j) == u[j])

#10
for i in D:
    mdl.add_constraint(mdl.sum(r[i,j] for j in D_n1 if i != j) == u[i])

# #11
for i in D:
    mdl.add_constraint(mdl.sum(y[i,j]*a[j] for j in N) <= A*u[i])

# #12
for j in N:
    mdl.add_constraint(mdl.sum(y[i,j] for i in D_0) == 1)

# #13
for j in N:
    mdl.add_constraint(T[j] >= w[j] + min(d[j], t[0,j]))

# # #14
for i in D_0:
    for j in N :
        if j == i:continue
        mdl.add_constraint(T[j] >= T[i] - M * (1 - y[i,j]))

# # #15
for i in N_0:
    for j in N_n1:
        if ((j in D) or (j == i)): continue
        mdl.add_constraint(T[j] >= T[i] + t[i,j] - M * (1 - x[i,j]))

# # #16
for i in N_0:
    for j in D :
        if j == i:continue
        mdl.add_constraint(T[j] >= T[i] + t[i,j] + delta*u[j] - M * (1 - x[i,j]))

# #17
for j in D:
    mdl.add_constraint(T[j] >= s[j] + d[j] + delta - M*(1 - u[j]))

# #18
for i in D:
    for j in D:
        if i == j: continue
        mdl.add_constraint(s[j] >= T[i] + d[i] - M * (1 - r[i,j]))

# # #19
for i in N:
    for j in D:
        mdl.add_constraint(s[j] >= w[i]*y[j,i])

# #20
for j in N:
    mdl.add_constraint(T[0] >= w[j]*y[0,j])

# #21
for j in D: 
    for i in N_0:
        if i == j:continue
        mdl.add_constraint(e[j] >= T[j] - (T[i] + t[i,j]) - M * (1 - x[i,j]))

# #22
for j in D:
    mdl.add_constraint(e[j] <= M*u[j])

# #23
for i in N:
    mdl.add_constraint(T[n+1] >= T[i] + t[i,n+1])

# # #24
mdl.add_constraint(T[n+1] >= T[0] + mdl.sum(t[i,j]*x[i,j] for i in N_0n1 for j in N_0n1) + sum(e[i] for i in D))
mdl.minimize(T[n+1])
mdl.set_time_limit(10800)

start_time = time.time()
solution = mdl.solve()
end_time = time.time()
solve_time = end_time - start_time
print('Solve Time:', solve_time)
print(solution)