

from math import radians, degrees
import numpy as np
import matplotlib.pyplot as plt



# Ocean current
current_vel = 0
current_hdg = 0

# Disturbance
dist_var = 0



def boat_dynamics(x,ctrl,verbose=False):
    

    # States are [surge velocity, sway velocity, yaw rate, x coord, y coord, yaw angle]^T
    u = x[0]    # m/s
    v = x[1]    # m/s
    r = x[2]    # rad/s
    x_b = x[3]  # m
    y_b = x[4]  # m
    yaw = x[5]  # rad

    # Control inputs are [rpm, fin angle, disturbance, water current magnitude, water current direction]^T
    prop_rpm = ctrl[0]  # RPM
    rudder = ctrl[1]    # rad
    dist = ctrl[2]      # rad
    # water_mag = ctrl[3]
    # water_dir = ctrl[4]


    # Apply rudder disturbance
    rudder = rudder + dist

    # Ensure rudder stays within limits
    max_rudder = radians(20)
    rudder = min(max_rudder, max(rudder, -max_rudder))


    ###############
    # Parameters

    # Vehicle mass, inertia
    m = 50 # mass in kg
    I_z = 15 # inertia 


    # Vehicle drag
    D_u = 10    # Forward drag coefficient
    D_v = 200   # Sideways drag coefficient


    # Rudder drag
    D_rud = 1   # Rudder drag coefficient
    D_u_rudder = D_rud * np.sin(rudder)
    D_v_rudder = D_rud * np.cos(rudder)

    # Rudder parameters
    M_rudder = 5    # Rudder moment coefficient

    # Propellor stats
    max_thrust = 100 # N
    max_rpm = 5000 # RPM

    


    ##############
    # Calculation


    # Rudder moment
    tau_rudder = M_rudder * u * abs(u) * rudder


    # Drag is based on speed
    u_drag = (D_u * abs(u) + D_u_rudder * abs(u)) * u
    v_drag = D_v * abs(v)*v + D_v_rudder * abs(u)*u
    

    # Thrust linear model from RPM
    if prop_rpm > max_rpm:
        prop_rpm = max_rpm
    elif prop_rpm < 0:
        prop_rpm = 0

    thrust = max_thrust * prop_rpm / max_rpm


    # Logging
    if verbose:
        # Control
        # print(f"RPM: {prop_rpm}")

        # Forces
        # print(f"Thrust: {thrust} | u_drag: {u_drag} | v_drag: {v_drag}")

        # Moments
        print(f"Moment: {tau_rudder}")


    # Body-frame update
    u_dot = (1/m) * (thrust - u_drag)
    v_dot = (1/m) * (-v_drag - tau_rudder)
    r_dot = (1/I_z) * tau_rudder

    # Global frame update
    x_dot = np.sqrt((u * np.cos(yaw))**2 + (v * np.sin(yaw))**2)
    y_dot = np.sqrt((u * np.sin(yaw))**2 + (v * np.cos(yaw))**2)
    yaw_dot = r


    # Return the update
    dx = np.array([u_dot, v_dot, r_dot, x_dot, y_dot, yaw_dot])
    return dx





def simulate_boat():

    # Setpoints
    yaw_setpoint = radians(45)
    u_setpoint = 2 # m/s

    # Controllers
    # Speed PID
    u_Ki = 1000
    u_integrator = 0

    # Yaw PID
    yaw_Kp = 0.25
    yaw_Ki = 0.0001
    yaw_Kd = -0.5
    yaw_integrator = 0


    # Simulation timing
    Tf = 30 #sec
    dt = 0.1
    n_times = round(Tf / dt)
    t_sim = np.linspace(0, Tf, n_times)

    # Initial condtions
    x0 = np.zeros((6,1))

    # Trajectory
    yout = np.zeros((6,n_times))
    uout = np.zeros((3,n_times))

    # Initialize
    x = x0
    yout[:,0] = np.squeeze(x)

    # Simulate
    for t_idx in range(n_times-1):
        # Current state
        u_k = yout[0,t_idx]
        r_k = yout[2,t_idx]
        yaw_k = yout[5,t_idx]

        # Control the boat
        # Speed
        u_error = (u_setpoint - u_k)
        u_integrator += u_error * dt
        rpm = u_Ki * u_integrator

        # Yaw
        yaw_error = yaw_setpoint - yaw_k
        yaw_integrator += yaw_error * dt
        rudder = (yaw_Kp * yaw_error) + (yaw_Ki * yaw_integrator) + (yaw_Kd * r_k)

        # Open loop test
        # rpm = 1000
        # rudder = 0
        dist_u = 0

        u = np.array([rpm, rudder, dist_u])

        # Simulate dynamics
        dx = boat_dynamics(x, u, verbose=False)

        # Update state
        x =  x + dx * dt

        # Save control
        uout[:,t_idx+1] = np.squeeze(u)

        # Save state
        yout[:,t_idx+1] = np.squeeze(x)
        

    # # Setpoint line
    u_setpoint_line = u_setpoint * np.ones(np.size(t_sim))
    yaw_setpoint_line = yaw_setpoint * np.ones(np.size(t_sim))

    # Plot Rates
    fig, axs = plt.subplots(3)
    axs[0].plot(t_sim, yout[0,:])
    # axs[0].plot(t, y_setpoint)
    axs[0].set_ylabel("Surge Velocity")

    axs[1].plot(t_sim, yout[1,:])
    # axs[1].plot(t, yout[2,:])
    axs[1].set_xlabel("Time (sec)")
    axs[1].set_ylabel("Sway Velocity")

    axs[2].plot(t_sim, yout[2,:])
    # axs[1].plot(t, yout[2,:])
    axs[2].set_xlabel("Time (sec)")
    axs[2].set_ylabel("Yaw Rate")
    
    # Save the figure as PNG
    plt.savefig(f'boat_rates.png', bbox_inches='tight')


    # Plot the position
    plt.figure()
    plt.plot(yout[3,:], yout[4,:])
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")

    plt.savefig(f'boat_map.png', bbox_inches='tight')

    
    # Plot control goals
    fig, axs = plt.subplots(2)
    axs[0].plot(t_sim, yout[0,:])
    axs[0].plot(t_sim, u_setpoint_line)
    axs[0].set_ylabel("Surge Velocity")

    axs[1].plot(t_sim, uout[0,:])
    axs[1].set_ylabel("RPM Command")
    axs[1].set_xlabel("Time (sec)")

    plt.savefig(f'boat_speed_control.png', bbox_inches='tight')

    fig, axs = plt.subplots(2)
    axs[0].plot(t_sim, np.degrees(yout[5,:]))
    axs[0].plot(t_sim, np.degrees(yaw_setpoint_line))
    axs[0].set_ylabel("Yaw (deg)")

    axs[1].plot(t_sim, np.degrees(uout[1,:]))
    axs[1].set_ylabel("Rudder (deg)")
    axs[1].set_xlabel("Time (sec)")

    plt.savefig(f'boat_yaw_control.png', bbox_inches='tight')



    # Show the plots (optional)
    # plt.show()

    



if __name__ == "__main__":
    # Control design
    simulate_boat()