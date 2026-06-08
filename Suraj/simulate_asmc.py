import numpy as np
import matplotlib.pyplot as plt
from asmc_omburo import ASMC_Balancer

def run_simulation():
    # =========================================================================
    # 1. SETUP THE "TRUE" VIRTUAL PLANT
    # We intentionally make the real robot heavier and less responsive 
    # than the controller expects, to prove the adaptive law works.
    # =========================================================================
    a21_true = 45.0  # Real robot is more top-heavy (Controller thinks 35.0)
    a22_true = 0.2   # Real robot has less friction (Controller thinks 0.5)
    b21_true = 2.0   # Real motors are slightly weaker (Controller thinks 2.5)

    # Simulation parameters
    dt = 0.002       # 500 Hz
    t_end = 5.0      # Simulate 5 seconds total
    time_steps = np.arange(0, t_end, dt)
    
    # State variables [theta, theta_dot]
    theta = 0.05     # Start slightly tipped over (50 milliradians)
    theta_dot = 0.0
    
    # Initialize the controller
    controller = ASMC_Balancer()
    
    # Data logging arrays for plotting
    log_theta = []
    log_theta_dot = []
    log_torque = []
    log_K_hat = []
    log_disturbance = []

    print("Starting purely mathematical ASMC simulation...")

    # =========================================================================
    # 2. MAIN SIMULATION LOOP (Euler Integration)
    # =========================================================================
    for t in time_steps:
        # A. Inject a Virtual Push (Disturbance)
        # At exactly 2.0 seconds, a human kicks the robot for 0.1 seconds
        if 2.0 <= t <= 2.1:
            external_disturbance_torque = 15.0 # A harsh 15 Nm kick
        else:
            external_disturbance_torque = 0.0
            
        # B. Controller Calculates Torque
        # The controller only knows the measured states, not the true physics
        commanded_torque, current_K, s = controller.compute_torque(theta, theta_dot, dt)
        
        # C. The True Physics Engine Reacts
        # True Acceleration = a21*theta + a22*theta_dot + b21*(Commanded + Disturbance)
        theta_ddot = (a21_true * theta) + \
                     (a22_true * theta_dot) + \
                     (b21_true * (commanded_torque + external_disturbance_torque))
        
        # Integrate acceleration to get velocity and position (dt = 0.002)
        theta_dot += theta_ddot * dt
        theta += theta_dot * dt
        
        # D. Log the data for this time step
        log_theta.append(theta)
        log_theta_dot.append(theta_dot)
        log_torque.append(commanded_torque)
        log_K_hat.append(current_K)
        log_disturbance.append(external_disturbance_torque)

    print("Simulation complete. Generating plots...")

    # =========================================================================
    # 3. PLOT THE RESULTS
    # =========================================================================
    fig, axs = plt.subplots(4, 1, figsize=(10, 10), sharex=True)
    fig.suptitle("ASMC Unicycle Balance & Push Recovery Simulation", fontsize=14)

    # Plot 1: Tilt Angle (Goal is to keep this at 0)
    axs[0].plot(time_steps, np.degrees(log_theta), 'b-', linewidth=2)
    axs[0].axhline(0, color='black', linestyle='--')
    axs[0].set_ylabel("Tilt Angle (Degrees)")
    axs[0].grid(True)

    # Plot 2: Disturbance (The Human Kick)
    axs[1].plot(time_steps, log_disturbance, 'r-', linewidth=2)
    axs[1].set_ylabel("Ext. Push (Nm)")
    axs[1].grid(True)

    # Plot 3: Controller Adaptive Gain (K_hat)
    axs[2].plot(time_steps, log_K_hat, 'g-', linewidth=2)
    axs[2].set_ylabel("Adaptive Gain (K_hat)")
    axs[2].grid(True)

    # Plot 4: Commanded Motor Torque
    axs[3].plot(time_steps, log_torque, 'm-', linewidth=2)
    axs[3].set_ylabel("Motor Command (Nm)")
    axs[3].set_xlabel("Time (seconds)")
    axs[3].grid(True)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    run_simulation()