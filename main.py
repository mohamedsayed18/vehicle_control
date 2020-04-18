"""Simulate the vehicle"""
import matplotlib.pyplot as plt

from way_points import waypoints
from Vehicle import Vehicle

# Read the way points
waypoints = waypoints()
way_x = [wp[0] for wp in waypoints]     # x points
way_y = [wp[1] for wp in waypoints]     # y points
x_data = []     # store car x
y_data = []     # store xr

# Create Model of the car
car = Vehicle()
# set the first point
car.x, car.y = waypoints[0][0], waypoints[0][1]

# Pass the output to the model (Step)
for point in waypoints:
    throttle = car.get_throttle(point)  # Calculate the throttle
    steer = car.lateral_control(point)  # calculate the steer angle
    car.step(throttle, steer)    # Give the car the throttle and steer
    x_data.append(car.x)    # add x position to the list
    y_data.append(car.y)    # add y position to the list

# Plot the trajectory
plt.plot(x_data, y_data, linewidth=0.5)
plt.show()
