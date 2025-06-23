import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import seaborn as sns

# Set up the plot style
plt.style.use('seaborn-v0_8')
sns.set_palette("husl")

# Simulation parameters
time_duration = 10  # seconds
dt = 0.004  # 4ms loop time (250 Hz)
time = np.arange(0, time_duration, dt)

# Target altitude
target_altitude = 100  # cm

# Create realistic noisy altitude data
np.random.seed(42)  # For reproducible results

# Base altitude with some drift (like wind or battery voltage changes)
base_drift = 2 * np.sin(0.3 * time) + 1 * np.cos(0.1 * time)

# High frequency sensor noise (realistic VL53L0X/BMP280 noise)
sensor_noise = np.random.normal(0, 1.5, len(time))  # ±1.5cm noise

# Occasional larger disturbances (like wind gusts)
disturbances = np.zeros(len(time))
for i in range(0, len(time), 1000):  # Every 4 seconds
    if i + 200 < len(time):
        disturbances[i:i+200] = 3 * np.random.normal(0, 1)

# Raw noisy altitude reading
raw_altitude = target_altitude + base_drift + sensor_noise + disturbances

# Apply low-pass filter (like our code: 85% old + 15% new)
filtered_altitude = np.zeros(len(time))
filtered_altitude[0] = raw_altitude[0]
alpha = 0.85  # Filter coefficient

for i in range(1, len(time)):
    filtered_altitude[i] = alpha * filtered_altitude[i-1] + (1 - alpha) * raw_altitude[i]

# Calculate errors (before deadband)
raw_error = target_altitude - raw_altitude
filtered_error = target_altitude - filtered_altitude

# Apply deadband function
def apply_deadband(error, deadband=2.0):
    """Apply deadband to error signal"""
    deadband_error = np.copy(error)
    deadband_error[np.abs(error) < deadband] = 0
    return deadband_error

deadband_value = 2.0  # 2cm deadband
raw_error_deadband = apply_deadband(raw_error, deadband_value)
filtered_error_deadband = apply_deadband(filtered_error, deadband_value)

# Calculate PID outputs (simplified - just proportional for demo)
Kp = 3.5  # From our code
raw_pid_output = Kp * raw_error_deadband
filtered_pid_output = Kp * filtered_error_deadband

# Create the comprehensive plot
fig, axes = plt.subplots(2, 2, figsize=(15, 12))
fig.suptitle('Altitude Hold: Deadband and Filtering Effects', fontsize=16, fontweight='bold')

# Plot 1: Raw vs Filtered Altitude
ax1 = axes[0, 0]
ax1.plot(time, raw_altitude, 'lightcoral', alpha=0.7, linewidth=1, label='Raw Noisy Sensor')
ax1.plot(time, filtered_altitude, 'darkblue', linewidth=2, label='Filtered Altitude')
ax1.axhline(y=target_altitude, color='green', linestyle='--', linewidth=2, label='Target (100cm)')

# Add deadband zone
deadband_upper = target_altitude + deadband_value
deadband_lower = target_altitude - deadband_value
ax1.fill_between(time, deadband_lower, deadband_upper, alpha=0.2, color='yellow', label=f'Deadband Zone (±{deadband_value}cm)')

ax1.set_xlabel('Time (seconds)')
ax1.set_ylabel('Altitude (cm)')
ax1.set_title('🎯 Altitude Readings: Raw vs Filtered')
ax1.legend()
ax1.grid(True, alpha=0.3)
ax1.set_ylim(92, 108)

# Plot 2: Error Signals
ax2 = axes[0, 1]
ax2.plot(time, raw_error, 'lightcoral', alpha=0.7, linewidth=1, label='Raw Error')
ax2.plot(time, filtered_error, 'darkblue', linewidth=2, label='Filtered Error')
ax2.axhline(y=deadband_value, color='red', linestyle=':', alpha=0.7, label=f'+{deadband_value}cm Deadband')
ax2.axhline(y=-deadband_value, color='red', linestyle=':', alpha=0.7, label=f'-{deadband_value}cm Deadband')
ax2.axhline(y=0, color='green', linestyle='-', alpha=0.5)

# Highlight deadband zone
ax2.fill_between(time, -deadband_value, deadband_value, alpha=0.2, color='yellow', label='Dead Zone')

ax2.set_xlabel('Time (seconds)')
ax2.set_ylabel('Error (cm)')
ax2.set_title('📏 Error Signals (Target - Actual)')
ax2.legend()
ax2.grid(True, alpha=0.3)
ax2.set_ylim(-8, 8)

# Plot 3: Error After Deadband
ax3 = axes[1, 0]
ax3.plot(time, raw_error_deadband, 'lightcoral', alpha=0.8, linewidth=2, label='Raw Error (with Deadband)')
ax3.plot(time, filtered_error_deadband, 'darkblue', linewidth=2, label='Filtered Error (with Deadband)')
ax3.axhline(y=0, color='green', linestyle='-', alpha=0.5)

ax3.set_xlabel('Time (seconds)')
ax3.set_ylabel('Active Error (cm)')
ax3.set_title('🚫 Errors After Deadband Application')
ax3.legend()
ax3.grid(True, alpha=0.3)
ax3.set_ylim(-8, 8)

# Plot 4: PID Controller Output (Throttle Correction)
ax4 = axes[1, 1]
ax4.plot(time, raw_pid_output, 'lightcoral', alpha=0.8, linewidth=1, label='Raw PID Output')
ax4.plot(time, filtered_pid_output, 'darkblue', linewidth=2, label='Filtered PID Output')
ax4.axhline(y=0, color='green', linestyle='-', alpha=0.5)

# Add constraint lines (from our code: ±300)
ax4.axhline(y=300, color='red', linestyle='--', alpha=0.7, label='Max Correction (+300)')
ax4.axhline(y=-300, color='red', linestyle='--', alpha=0.7, label='Max Correction (-300)')

ax4.set_xlabel('Time (seconds)')
ax4.set_ylabel('Throttle Correction')
ax4.set_title('🎮 PID Controller Output')
ax4.legend()
ax4.grid(True, alpha=0.3)

plt.tight_layout()

# Calculate and display statistics
print("📊 ALTITUDE HOLD PERFORMANCE STATISTICS")
print("=" * 50)

# RMS (Root Mean Square) of errors - lower is better
raw_rms = np.sqrt(np.mean(raw_error**2))
filtered_rms = np.sqrt(np.mean(filtered_error**2))

print(f"🎯 RMS Error (lower = better):")
print(f"   Raw sensor:      {raw_rms:.2f} cm")
print(f"   Filtered:        {filtered_rms:.2f} cm")
print(f"   Improvement:     {((raw_rms - filtered_rms)/raw_rms)*100:.1f}%")

# Count how often controller is active (error > deadband)
raw_active = np.sum(np.abs(raw_error) > deadband_value)
filtered_active = np.sum(np.abs(filtered_error) > deadband_value)
total_samples = len(time)

print(f"\n⚡ Controller Activity (less = smoother):")
print(f"   Raw sensor:      {(raw_active/total_samples)*100:.1f}% of time")
print(f"   Filtered:        {(filtered_active/total_samples)*100:.1f}% of time")
print(f"   Reduction:       {((raw_active - filtered_active)/raw_active)*100:.1f}%")

# Maximum corrections
raw_max_correction = np.max(np.abs(raw_pid_output))
filtered_max_correction = np.max(np.abs(filtered_pid_output))

print(f"\n🚁 Maximum Throttle Corrections:")
print(f"   Raw sensor:      ±{raw_max_correction:.1f}")
print(f"   Filtered:        ±{filtered_max_correction:.1f}")
print(f"   Reduction:       {((raw_max_correction - filtered_max_correction)/raw_max_correction)*100:.1f}%")

# Deadband effectiveness
raw_corrections = np.sum(raw_error_deadband != 0)
filtered_corrections = np.sum(filtered_error_deadband != 0)

print(f"\n🚫 Deadband Effectiveness:")
print(f"   Raw corrections:     {raw_corrections} samples")
print(f"   Filtered corrections: {filtered_corrections} samples")
print(f"   Avoided corrections:  {raw_corrections - filtered_corrections} samples")

plt.show()

# Create a separate zoomed-in plot to show detail
fig2, ax = plt.subplots(1, 1, figsize=(12, 6))

# Focus on a 2-second window to show detail
start_idx = int(3 / dt)  # Start at 3 seconds
end_idx = int(5 / dt)    # End at 5 seconds
time_zoom = time[start_idx:end_idx]

ax.plot(time_zoom, raw_altitude[start_idx:end_idx], 'lightcoral', alpha=0.8, linewidth=1, 
        label='Raw Noisy Sensor', marker='o', markersize=1)
ax.plot(time_zoom, filtered_altitude[start_idx:end_idx], 'darkblue', linewidth=3, 
        label='Filtered Altitude (Our System)')
ax.axhline(y=target_altitude, color='green', linestyle='--', linewidth=2, label='Target Altitude')

# Add deadband zone
ax.fill_between(time_zoom, deadband_lower, deadband_upper, alpha=0.3, color='yellow', 
                label=f'Deadband Zone (±{deadband_value}cm)')

ax.set_xlabel('Time (seconds)')
ax.set_ylabel('Altitude (cm)')
ax.set_title('🔍 Detailed View: How Filtering Tames Sensor Noise (2-second window)')
ax.legend()
ax.grid(True, alpha=0.3)
ax.set_ylim(96, 104)

plt.tight_layout()
plt.show()

print("\n" + "="*60)
print("🎯 KEY TAKEAWAYS:")
print("="*60)
print("✅ FILTERING removes high-frequency noise")
print("✅ DEADBAND prevents micro-corrections") 
print("✅ COMBINATION = smooth, stable altitude hold")
print("✅ MOTORS stay quiet instead of constantly buzzing")
print("✅ BATTERY lasts longer due to less motor activity")
print("="*60)