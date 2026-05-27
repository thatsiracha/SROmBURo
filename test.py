import time
from Omburo import Omburo

# Initialize Omburo
omburo = Omburo()

print("Starting Omburo test sequence...")
print("=" * 60)

# Enable torque
print("Enabling torque...")
omburo.toggleTorque(1)
time.sleep(0.5)

# Apply 0.1 N-m torque for 5 seconds
print("Applying 0.1 N-m torque for 5 seconds...")
print("Reading joint position and velocity:")
print("-" * 60)

omburo.setTorque(0.1, 0.1)

start_time = time.time()
while time.time() - start_time < 5.0:
    position_wheel, velocity_wheel, position_roller, velocity_roller = omburo.readback()
    elapsed = time.time() - start_time
    print(f"[{elapsed:.2f}s] Wheel - Pos: {position_wheel:.4f}, Vel: {velocity_wheel:.4f} | "
          f"Roller - Pos: {position_roller:.4f}, Vel: {velocity_roller:.4f}")
    time.sleep(0.1)  # Read every 100ms

# Disable torque
print("-" * 60)
print("Disabling torque...")
omburo.toggleTorque(0)
time.sleep(0.5)

# Close the port
print("Closing Omburo port...")
omburo.close()

print("=" * 60)
print("Test sequence completed successfully!")
