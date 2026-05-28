import numpy as np
import time

from Omburo import Omburo

#Define Kp, Ki, Kd

Kp = 1.0
Ki = 0.1
Kd = 0.05

# Initialize Omburo
omburo = Omburo()

print("Starting Omburo test sequence...")
print("=" * 60)

BEAR_connected = omburo.getError()
if not BEAR_connected:
    print("Error: Unable to connect to BEAR. Please check connections and try again.")
    exit(1)

# Enable torque
print("Enabling torque...")
omburo.toggleTorque(1)

ErrorX = 
ErrorY = 