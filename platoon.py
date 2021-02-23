import matplotlib.pyplot as plt
import numpy as np
import math
import cubic_spline_planner

# Vehicle parameters
PARA = 1 #Use to minimize the vehicle parameters
LENGTH = 4.5/PARA  # [m]
WIDTH = 2.0/PARA  # [m]
BACKTOWHEEL = 1.0/PARA  # [m]
WHEEL_LEN = 0.3/PARA  # [m]
WHEEL_WIDTH = 0.2/PARA  # [m]
TREAD = 0.7/PARA  # [m]
WB = 2.5/PARA  # [m]


# start positions
NUM_CARS = 4 #NUMBER OF CARS
MAX_CARS = 7
X_START1 = 60
X_START2 = 50
X_START3 = 40
X_START4 = 30
X_START5 = 20
X_START6 = 10
X_START7 = 0
X_LIST = [X_START1, X_START2, X_START3, X_START4, X_START5, X_START6, X_START7]

# Constrains on accelaration
U_MIN = -1.5
U_MAX = 1.5
U_INIT = 0

# Constrains on velocity
V_MAX = 120
V_MIN = 0
V_INIT = 60

#Constrains on distance
S0 = 1 # 1 meter between the cars is the minimum possible.

# time step
DT = 0.1/6

#Parameters for cost function
C1 = 0.1
C2 = 1
C3 = 0.


# initialy distance between the cars
S_REF = X_START1 - X_START2
# reference distance when vehicles should split
S_REF_SPLIT = 2 * S_REF


def plot_car(x, y, yaw, steer=0.0, cabcolor="-r", truckcolor="-k"):  # pragma: no cover

    outline = np.array([[-BACKTOWHEEL, (LENGTH - BACKTOWHEEL), (LENGTH - BACKTOWHEEL), -BACKTOWHEEL, -BACKTOWHEEL],
                        [WIDTH / 2, WIDTH / 2, - WIDTH / 2, -WIDTH / 2, WIDTH / 2]])

    fr_wheel = np.array([[WHEEL_LEN, -WHEEL_LEN, -WHEEL_LEN, WHEEL_LEN, WHEEL_LEN],
                         [-WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD]])

    rr_wheel = np.copy(fr_wheel)

    fl_wheel = np.copy(fr_wheel)
    fl_wheel[1, :] *= -1
    rl_wheel = np.copy(rr_wheel)
    rl_wheel[1, :] *= -1

    Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                     [-math.sin(yaw), math.cos(yaw)]])
    Rot2 = np.array([[math.cos(steer), math.sin(steer)],
                     [-math.sin(steer), math.cos(steer)]])

    fr_wheel = (fr_wheel.T.dot(Rot2)).T
    fl_wheel = (fl_wheel.T.dot(Rot2)).T
    fr_wheel[0, :] += WB
    fl_wheel[0, :] += WB

    fr_wheel = (fr_wheel.T.dot(Rot1)).T
    fl_wheel = (fl_wheel.T.dot(Rot1)).T

    outline = (outline.T.dot(Rot1)).T
    rr_wheel = (rr_wheel.T.dot(Rot1)).T
    rl_wheel = (rl_wheel.T.dot(Rot1)).T

    outline[0, :] += x
    outline[1, :] += y
    fr_wheel[0, :] += x
    fr_wheel[1, :] += y
    rr_wheel[0, :] += x
    rr_wheel[1, :] += y
    fl_wheel[0, :] += x
    fl_wheel[1, :] += y
    rl_wheel[0, :] += x
    rl_wheel[1, :] += y

    plt.plot(np.array(outline[0, :]).flatten(),
             np.array(outline[1, :]).flatten(), truckcolor)
    plt.plot(np.array(fr_wheel[0, :]).flatten(),
             np.array(fr_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(rr_wheel[0, :]).flatten(),
             np.array(rr_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(fl_wheel[0, :]).flatten(),
             np.array(fl_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(rl_wheel[0, :]).flatten(),
             np.array(rl_wheel[1, :]).flatten(), truckcolor)
    plt.plot(x, y, "*")

def get_straight_course(dl):
    a = 5 # A parameter to make the trajectory longer
    ax = [a*0.0, a*5.0, a*10.0, a*20.0, a*30.0, a*40.0, a*50.0]
    ay = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)

    return cx, cy, cyaw, ck

def distance(distance_list, temp_disp_list):
    '''Calculates the distance between the ancestor car'''
    new_distance_list = []
    for i in range(NUM_CARS - 1):
        delta = temp_disp_list[-NUM_CARS] - temp_disp_list[-NUM_CARS + i]
        new_distance_list.append(distance_list[-NUM_CARS + 1 + i] + delta)
    for m in range(len(new_distance_list)):
        distance_list.append(new_distance_list[m])
    return distance_list

def clear_and_draw_car(x_list):
    plt.cla()
    dl = 1  # course tick
    cx, cy, cyaw, ck = get_straight_course(dl)  # get the straight line
    plt.plot(cx, cy, "-r", label="course")
    for i in range(NUM_CARS):
        plot_car(x_list[-1-i], 0, 0, steer=0.0) # plot the cars

    plt.axis([0, 300, -50, 50])
    plt.grid(True)
    plt.pause(0.0001)

def velocity_plotter(dt_list, v_list):
    plt.subplots(1)
    colors = ['-b', '-g', '-r', '-c', '-m', '-y', '-k']
    labels = ['v1', 'v2', 'v3', 'v4', 'v5', 'v6', 'v7']

    for i in range(NUM_CARS):
        plt.plot(dt_list, v_list[i::NUM_CARS], colors[i], label = labels[i])
    plt.grid(True)
    plt.axis("equal")
    plt.xlabel("time(s)")
    plt.ylabel("velocity")
    plt.legend()
    plt.show()

def acceleration_plotter(dt_list, u_list):
    plt.subplots(1)
    colors = ['-b', '-g', '-r', '-c', '-m', '-y', '-k']
    labels = ['a1', 'a2', 'a3', 'a4', 'a5', 'a6', 'a7']

    for i in range(NUM_CARS):
        plt.plot(dt_list, u_list[i::NUM_CARS], colors[i], label = labels[i])
    plt.grid(True)
    plt.axis("equal")
    plt.xlabel("time(s)")
    plt.ylabel("acceleration(m/s^2)")
    plt.legend()
    plt.show()

def distance_plotter(dt_list, distance_list):
    plt.subplots(1)
    colors = ['-b', '-g', '-r', '-c', '-m', '-y', '-k']
    labels = ['Δ12', 'Δ23', 'Δ34', 'Δ45', 'Δ56', 'Δ67']
    for i in range(NUM_CARS-1):
        plt.plot(dt_list, distance_list[i::NUM_CARS-1], colors[i], label = labels[i])
    plt.grid(True)
    plt.xlabel("time(s)")
    plt.ylabel("distance(m)")
    plt.legend()
    plt.show()

def old_values_lists():
    ''' Define list to hold old values for plotting the graphs '''

    # Initial car positions
    x_list = []
    for i in range(NUM_CARS):
        x_list.append(X_LIST[MAX_CARS-NUM_CARS+i])

    # Initial velocities
    v_list = []
    for i in range(NUM_CARS):
        v_list.append(V_INIT)

    # Initial accelerations
    u_list = []
    for i in range(NUM_CARS):
        u_list.append(U_INIT)

    # Distance between the cars at the begining
    distance_list = []
    for i in range(NUM_CARS-1):
        distance_list.append(x_list[i] - x_list[i+1])

    # Time list
    dt_list = [0]

    return x_list, v_list, u_list, distance_list, dt_list

def position_plotter(dt_list, x_list):
    plt.subplots(1)
    colors = ['-b', '-g', '-r', '-c', '-m', '-y', '-k']
    labels = ['x1', 'x2', 'x3', 'x4', 'x5', 'x6', 'x7']

    for i in range(NUM_CARS):
        plt.plot(dt_list, x_list[i::NUM_CARS], colors[i], label=labels[i])
    plt.grid(True)
    plt.xlabel("time(s)")
    plt.ylabel("position(m)")
    plt.legend()
    plt.show()

def animation():
    '''Does the required calculations and plots the animation.'''

    # Coordinate, velocity and acceleration lists. These are for plotting.
    x_list, v_list, u_list, distance_list, dt_list = old_values_lists()

    for i in range(200):
        clear_and_draw_car(x_list)

        # If the accelartion is different from 0 the velocity for the car should change according to a = dv/dt -> dv = a*dt
        # And the total velocitiy is then v_new = v_old + dt
        for i in range(NUM_CARS):
            v_new = u_list[-NUM_CARS+ i] * DT + v_list[-NUM_CARS]
            v_list.append(v_new)

        for i in range(NUM_CARS):
            u_new = 0 # Here we should add some call to MPC function
            u_list.append(u_new)

        dt_list.append(DT+dt_list[-1])

        #The displacement in 1 iteration is calculated by x = v*dt
        # And the new position is calculated by the old position + the displacement

        temp_disp_list = []
        for i in range(NUM_CARS):
            displacement = v_list[-NUM_CARS + i] * DT
            temp_disp_list.append(displacement)
            x_new = x_list[-NUM_CARS] + displacement
            x_list.append(x_new)

        distance_list = distance(distance_list, temp_disp_list)

    velocity_plotter(dt_list, v_list)
    distance_plotter(dt_list, distance_list)
    acceleration_plotter(dt_list, u_list)
    position_plotter(dt_list, x_list)

def main():
    animation()

main()

