import time
import math
import threading
import serial
import time
import math
import imu_read  # This imports your unmodified IMU script
import Omburo

class IMU_Bridge:
    def __init__(self, port="/dev/ttyACM0", baudrate=115200):
        self.theta = 0.0
        self.theta_dot = 0.0
        self.running = True
        
        print(f"Initializing IMU Bridge on {port}...")
        self.ser = serial.Serial(port, baudrate, timeout=0.05, dsrdtr=False, rtscts=False)
        time.sleep(1.5)
        self.ser.reset_input_buffer()
        
        # Ping the device using your script's function
        if not imu_read.send_and_ack(self.ser, imu_read.cmd_ping(), 0x01, "Ping"):
            raise RuntimeError("IMU failed to respond to Ping.")

        # Request Accel, Gyro, and Euler fields
        fields = [imu_read.FIELD_ACCEL, imu_read.FIELD_GYRO, imu_read.FIELD_EULER]
        
        # NOTE: I bumped the sample rate to 500 Hz here to match your ASMC control loop speed!
        imu_read.send_and_ack(self.ser, imu_read.cmd_set_imu_format(500, fields), 0x08, "IMU format")
        imu_read.send_and_ack(self.ser, imu_read.cmd_enable_imu_stream(True), 0x11, "Start stream")
        
        # Start the background polling thread
        self.thread = threading.Thread(target=self._poll_imu_stream, daemon=True)
        self.thread.start()
        print("IMU background thread actively buffering data.")

    def _poll_imu_stream(self):
        """ Runs continuously in the background parsing serial packets. """
        while self.running:
            pkt = imu_read.read_packet(self.ser, timeout=0.1)
            if pkt:
                desc_set, payload = pkt
                if desc_set == imu_read.DESC_IMU:
                    data = imu_read.parse_imu_packet(payload)
                    
                    # 1. Extract Pitch (theta)
                    # Your script parses Euler to degrees. The ASMC math requires radians.
                    if "euler_deg" in data:
                        roll, pitch, yaw = data["euler_deg"]
                        self.theta = math.radians(pitch) 
                        
                    # 2. Extract Pitch Velocity (theta_dot)
                    # Gyro is already in rad/s. 
                    # *Important:* You may need to change 'gy' to 'gx' depending on 
                    # which way the IMU is physically mounted on your robot.
                    if "gyro_rads" in data:
                        gx, gy, gz = data["gyro_rads"]
                        self.theta_dot = gy  

    def read_imu(self):
        """ Instant, non-blocking fetch of the latest state for the ASMC loop. """
        return self.theta, self.theta_dot

    def close(self):
        """ Safely shuts down the stream and serial port. """
        self.running = False
        self.ser.write(imu_read.cmd_enable_imu_stream(False))
        self.ser.close()
        self.thread.join()
        
# ==============================================================================
# HARDWARE INTERFACE PLACEHOLDERS (TODO: Replace with your actual libraries)
# ==============================================================================


def send_torque_to_bear_motor(torque_nm):
    """ Send torque command via CAN to Bear motor drivers. """
    # Example: can_bus.send(ID, torque_nm)
    pass

# ==============================================================================
# ADAPTIVE SLIDING MODE CONTROLLER CLASS
# ==============================================================================
class ASMC_Balancer:
    def __init__(self):
        # ---------------------------------------------------------
        # 1. PHYSICS NOMINAL MODEL (TODO: Modify these rough guesses)
        # ---------------------------------------------------------
        # These are the a21, a22, and b21 terms from your A and B matrices.
        # They don't need to be perfect; the adaptive gain will fix errors.
        self.a21 = 35.0  # Gravity tipping parameter (roughly g / length_to_COM)
        self.a22 = 0.5   # Baseline mechanical damping/friction
        self.b21 = 2.5   # Control authority (roughly 1 / Total_Inertia)

        # ---------------------------------------------------------
        # 2. CONTROLLER TUNING PARAMETERS (TODO: Tune these on hardware)
        # ---------------------------------------------------------
        self.lambda_param = 15.0      # How fast it tries to return to vertical (s = 0)
        self.gamma = 25.0             # Adaptation rate (how fast K grows when pushed)
        self.phi_boundary = 0.1       # Boundary layer thickness (stops motor chattering)
        
        self.K_hat = 1.0              # Initial adaptive gain
        self.K_min = 0.5              # Minimum gain (keeps motors smooth when idle)
        self.K_max = 15.0             # Maximum safety torque bound (Nm)

    def compute_torque(self, theta, theta_dot, dt):
        """ Calculates the total commanded torque for a given state. """
        
        # Step 1: Calculate the Sliding Surface (s)
        s = theta_dot + (self.lambda_param * theta)
        
        # Step 2: Adaptive Gain Update Law
        if abs(s) > self.phi_boundary:
            # Pushed outside boundary: Rapidly scale UP the gain to fight back
            self.K_hat += (self.gamma * abs(s) * dt)
            sat_s = 1.0 if s > 0 else -1.0
        else:
            # Inside boundary (Safe): Slowly scale DOWN the gain to keep motors cool
            self.K_hat -= (0.5 * self.K_hat * dt) 
            sat_s = s / self.phi_boundary
            
        # Enforce safety bounds on the adaptive gain
        self.K_hat = max(self.K_min, min(self.K_hat, self.K_max))
        
        # Step 3: Compute Equivalent Control (Model-based)
        # tau_eq = (-f(x) - lambda*theta_dot) / b21
        f_x = (self.a21 * theta) + (self.a22 * theta_dot)
        tau_eq = (-f_x - (self.lambda_param * theta_dot)) / self.b21
        
        # Step 4: Compute Switching Control (Adaptive push recovery)
        tau_sw = (-self.K_hat * sat_s) / self.b21
        
        # Step 5: Total Torque
        total_torque = tau_eq + tau_sw
        return total_torque, self.K_hat, s

# ==============================================================================
# MAIN REAL-TIME LOOP
# ==============================================================================
def main():
    print("Starting OmBURo ASMC Balancing Loop...")
    
    # 1. Initialize the threaded IMU Bridge
    imu = IMU_Bridge(port="/dev/ttyACM0")
    robot = Omburo()
    
    # Enable power to the motor coils
    robot.toggleTorque(1)

    # 2. Initialize the Controller
    controller = ASMC_Balancer()
    target_dt = 0.002 # 500 Hz
    
    try:
        while True:
            start_time = time.perf_counter()
            
            # --- Non-blocking fetch of latest IMU state ---
            theta, theta_dot = imu.read_imu()
            
            # Compute Control Law
            torque, current_K, s = controller.compute_torque(theta, theta_dot, target_dt)
            
            # --- C. Actuate ---
            # We are testing the main wheel first. 
            # Send the calculated torque to the wheel, and 0.0 to the roller.
            robot.setTorque(torque_wheel=torque, torque_roller=0.0)
            
            # --- D. Enforce Loop Timing ---
            elapsed = time.perf_counter() - start_time
            if elapsed < target_dt:
                time.sleep(target_dt - elapsed)
                
    except KeyboardInterrupt:
        print("\nCtrl+C detected. Safely shutting down hardware...")
        
        # 1. Command zero torque immediately
        robot.setTorque(0.0, 0.0)
        
        # 2. Disable motor coils to prevent runaway
        robot.toggleTorque(0)
        
        # 3. Close serial ports
        robot.close()
        imu.close()
        
        print("Shutdown complete.")

if __name__ == "__main__":
    main()