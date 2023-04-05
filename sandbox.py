#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt 
# read file occ_grid.txt
# create a 2D array of 0s and 1s
# 0 = free space
# 1 = occupied space

file = open("occ_grid.txt", "r")
lines = file.readlines()
file.close()

# create 2D grid
grid = np.zeros((len(lines), len(lines[0].strip())))
for i in range(len(lines)):
    for j in range(len(lines[i].strip())):
        if lines[i][j] == '1':
            grid[i][j] = 1
        else:
            grid[i][j] = 0
# convert to numpy array
grid = np.array(grid)
print(grid.shape)

# flip bottom to top
grid = np.flip(grid, axis=0)
plt.imshow(grid[:,:250], cmap='Greys')
plt.show()
