import time
import numpy as np
import math
import sys
from docplex.mp.model import Model
from src.data import readData, getTruckRoute

time_limit = 1000 * 60 * 30
truck_speed = 30/60
drone_speed = 60/60
A = 4
delta = 5


M = 100000
enduration = 90

n, w, t, d = readData("R101_1.5.dat", truck_speed, drone_speed)

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

#25
for i in N_0n1:
    for j in N_0n1:
        x[(i, j)] = mdl.binary_var(name=f'x_{i}_{j}')

#29
for i in N_0n1:
    T[i] = mdl.continuous_var(name=f'T_{i}')


#35
for i in N:
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

# #13
for j in N:
    mdl.add_constraint(T[j] >= w[j] + min(d[j], t[0,j]))

# #23
for i in N:
    mdl.add_constraint(T[n+1] >= T[i] + t[i,n+1])

#32
for i in N_0n1:
    for j in N_0n1:
        mdl.add_constraint(T[j] >= T[i] + t[i, j] - M*(1 - x[i, j]))

#33
for j in N:
    mdl.add_constraint(e[j] >= T[j] - (T[i] + t[i, j]) - M*(1 - x[i, j]))

#34
mdl.add_constraint(T[n+1] >= T[0] + mdl.sum(t[i,j]*x[i,j] for i in N_0n1 for j in N_0n1) + sum(e[i] for i in D))


mdl.minimize(T[n+1])
mdl.set_time_limit(10800)

start_time = time.time()
solution = mdl.solve()
end_time = time.time()
solve_time = end_time - start_time
print('Stage 1 -- Done')
print('Solve Time:', solve_time)
print(solution)

pi = getTruckRoute(x, solution, n).astype(int).tolist()

u = {}
T = {}
y = {}

I = N

mdl = Model('integer_programming')

for i in D:
    u[pi[i]] = mdl.binary_var(name=f'u_{pi[i]}')

for i in D_0:
    for j in I:
        if (j < i): continue
        y[pi[i], pi[j]] = mdl.binary_var(name=f'y_{pi[i]}_{pi[j]}')

for i in N_0:
    T[pi[i]] = mdl.continuous_var(name=f'T_{pi[i]}')

#37
for j in I:
    mdl.add_constraint(mdl.sum(y[pi[i], pi[j]] for i in D_0 if i <= j) == 1 )

for i in D:
    mdl.add_constraint(mdl.sum(y[pi[i], pi[j]] * a[pi[j]] for j in I if j >= i) <= A * u[pi[i]])

for j in I:
    mdl.add_constraint(T[pi[0]] >= w[pi[j]] * y[pi[0], pi[j]])

for i in I:
    if i in D: continue
    mdl.add_constraint(T[pi[i]] >= T[pi[i-1]] + t[pi[i-1], pi[i]])

for i in D:
    mdl.add_constraint(T[pi[i]] >= T[pi[i-1]] + t[pi[i-1], pi[i]] + delta * u[pi[i]])

for i in D:
    for j in I:
        if j < i: continue
        mdl.add_constraint(T[pi[i]] >= (w[pi[j]] + d[pi[i]] + delta) * y[(pi[i], pi[j])])

for i in D:
    if i == 1: continue
    for k in D:
        if k >=i: continue
        mdl.add_constraint(T[pi[i]] >= T[pi[k]] + d[pi[k]] + d[pi[i]] + delta - M * (2 - u[pi[i]] - u[pi[k]]))

mdl.minimize(T[pi[n]] + t[pi[n], n+1])
mdl.set_time_limit(10800)

start_time = time.time()
solution = mdl.solve()
end_time = time.time()
solve_time = end_time - start_time
print('Stage 2 -- Done')
print('Solve Time:', solve_time)   
print(solution[T[pi[n]] + t[pi[n], n+1]])





