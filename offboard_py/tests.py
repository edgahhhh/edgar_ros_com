import numpy as np

heading = np.random.uniform(0, 2) * np.pi

flat_dist = 2   # meters
x_position = 2 * np.cos(heading)
y_position = 2 * np.sin(heading)

print(f' \n\nx = {x_position} \ny = {y_position} \n\n')