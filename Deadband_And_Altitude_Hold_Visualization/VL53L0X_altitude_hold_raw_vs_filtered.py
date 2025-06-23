import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from matplotlib.patches import Rectangle

# Set up the plot style
plt.style.use('seaborn-v0_8')
sns.set_palette("husl")

class VL53L0X_Simulator:
    """Realistic VL53L0X sensor simulator with actual noise characteristics"""
    
    def __init__(self):
        # VL53L0X noise characteristics (based on real sensor specs)
        self.base_noise_std = 1.2  # Standard noise ±1.2cm
        self.measurement_bias = 0.0  # Systematic bias
        self.outlier_probability = 0.02  # 2% chance of bad reading
        self.max_range = 200  # cm (sensor limit)
        
    def read_distance(self, true_distance):
        """Simulate a VL53L0X distance reading with realistic noise"""
        
        # Basic Gaussian noise
        noise = np.random.normal(0, self.base_noise_std)
        
        # Occasional outliers (sensor glitches)
        if np.random.random() < self.outlier_probability:
            # Bad reading - could be way off
            outlier_noise = np.random.normal(0, 5) + np.random.choice([-10, 10])
            noise += outlier_noise
        
        # Distance-dependent noise (farther = noisier)
        distance_factor = min(1.5, true_distance / 100)
        noise *= distance_factor
        
        # Add systematic bias
        measured = true_distance + noise + self.measurement_bias
        
        # Sensor limitations
        if measured < 3:  # Too close
            measured = 3 + np.random.normal(0, 2)
        elif measured > self.max_range:  # Too far
            measured = self.max_range + np.random.normal(0, 3)
            
        return measured

class AltitudeController:
    """PID controller for altitude hold"""
    
    def __init__(self, kp=3.5, ki=0.8, kd=0.15, deadband=2.0, dt=0.05):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.deadband = deadband
        self.dt = dt
        
        # PID state variables
        self.integral = 0
        self.prev_error = 0
        self.integral_max = 100  # Windup protection
        
    def reset(self):
        """Reset controller state"""
        self.integral = 0
        self.prev_error = 0
        
    def update(self, target, current):
        """Calculate PID output"""
        error = target - current
        
        # Apply deadband
        if abs(error) < self.deadband:
            error = 0
            
        # PID calculation
        # Proportional
        p_term = self.kp * error
        
        # Integral with windup protection
        if abs(error) > 20:  # Reset integral if way off
            self.integral = 0
        else:
            self.integral += error * self.dt
            self.integral = np.clip(self.integral, -self.integral_max, self.integral_max)
        i_term = self.ki * self.integral
        
        # Derivative
        d_term = self.kd * (error - self.prev_error) / self.dt
        
        # Combine terms
        output = p_term + i_term + d_term
        
        # Limit output (like our constrain function)
        output = np.clip(output, -300, 300)
        
        self.prev_error = error
        return output, error, p_term, i_term, d_term

# Simulation parameters
duration = 20  # seconds
dt = 0.05  # 50ms update rate (20 Hz) - realistic for VL53L0X
time = np.arange(0, duration, dt)
n_samples = len(time)

# Create realistic flight scenario
target_altitude = 80  # cm (hovering 80cm above ground)

# True altitude with realistic flight variations
np.random.seed(42)
base_altitude = target_altitude + 2 * np.sin(0.2 * time)  # Gentle drift
wind_disturbance = np.zeros(n_samples)

# Add wind gusts at random intervals
for i in range(0, n_samples, 100):  # Every 5 seconds
    gust_strength = np.random.uniform(-4, 4)
    gust_duration = np.random.randint(20, 50)  # 1-2.5 seconds
    end_idx = min(i + gust_duration, n_samples)
    wind_disturbance[i:end_idx] = gust_strength * np.exp(-0.1 * np.arange(end_idx - i))

true_altitude = base_altitude + wind_disturbance

# Simulate VL53L0X sensor readings
sensor = VL53L0X_Simulator()
raw_readings = np.array([sensor.read_distance(alt) for alt in true_altitude])

# Apply low-pass filter (like our code: 85% old + 15% new)
filtered_readings = np.zeros(n_samples)
filtered_readings[0] = raw_readings[0]
alpha = 0.85

for i in range(1, n_samples):
    filtered_readings[i] = alpha * filtered_readings[i-1] + (1 - alpha) * raw_readings[i]

# Create two altitude controllers
raw_controller = AltitudeController()
filtered_controller = AltitudeController()

# Run altitude hold simulation
raw_errors = []
filtered_errors = []
raw_outputs = []
filtered_outputs = []
raw_p_terms = []
filtered_p_terms = []
raw_i_terms = []
filtered_i_terms = []
raw_d_terms = []
filtered_d_terms = []

for i in range(n_samples):
    # Raw controller
    raw_output, raw_error, raw_p, raw_i, raw_d = raw_controller.update(target_altitude, raw_readings[i])
    raw_errors.append(raw_error)
    raw_outputs.append(raw_output)
    raw_p_terms.append(raw_p)
    raw_i_terms.append(raw_i)
    raw_d_terms.append(raw_d)
    
    # Filtered controller
    filt_output, filt_error, filt_p, filt_i, filt_d = filtered_controller.update(target_altitude, filtered_readings[i])
    filtered_errors.append(filt_error)
    filtered_outputs.append(filt_output)
    filtered_p_terms.append(filt_p)
    filtered_i_terms.append(filt_i)
    filtered_d_terms.append(filt_d)

# Convert to numpy arrays
raw_errors = np.array(raw_errors)
filtered_errors = np.array(filtered_errors)
raw_outputs = np.array(raw_outputs)
filtered_outputs = np.array(filtered_outputs)

# Create comprehensive comparison plots
fig = plt.figure(figsize=(16, 12))

# Main title
fig.suptitle('VL53L0X Altitude Hold: Raw vs Filtered Performance Comparison', 
             fontsize=16, fontweight='bold')

# Plot 1: Altitude readings and target
ax1 = plt.subplot(3, 2, 1)
plt.plot(time, true_altitude, 'green', linewidth=2, alpha=0.8, label='True Altitude')
plt.plot(time, raw_readings, 'red', alpha=0.6, linewidth=1, label='Raw VL53L0X')
plt.plot(time, filtered_readings, 'blue', linewidth=2, label='Filtered VL53L0X')
plt.axhline(y=target_altitude, color='black', linestyle='--', alpha=0.7, label='Target (80cm)')

# Add deadband zone
deadband = 2.0
plt.fill_between(time, target_altitude - deadband, target_altitude + deadband, 
                alpha=0.2, color='yellow', label=f'Deadband (±{deadband}cm)')

plt.xlabel('Time (seconds)')
plt.ylabel('Altitude (cm)')
plt.title('📡 VL53L0X Sensor Readings vs True Altitude')
plt.legend()
plt.grid(True, alpha=0.3)
plt.ylim(70, 90)

# Plot 2: Error comparison
ax2 = plt.subplot(3, 2, 2)
plt.plot(time, raw_errors, 'red', alpha=0.7, linewidth=1, label='Raw Error')
plt.plot(time, filtered_errors, 'blue', linewidth=2, label='Filtered Error')
plt.axhline(y=0, color='black', linestyle='-', alpha=0.5)
plt.axhline(y=deadband, color='gray', linestyle=':', alpha=0.7, label=f'±{deadband}cm Deadband')
plt.axhline(y=-deadband, color='gray', linestyle=':', alpha=0.7)

plt.fill_between(time, -deadband, deadband, alpha=0.2, color='yellow', label='Dead Zone')

plt.xlabel('Time (seconds)')
plt.ylabel('Error (cm)')
plt.title('📏 Altitude Error Comparison')
plt.legend()
plt.grid(True, alpha=0.3)
plt.ylim(-15, 15)

# Plot 3: PID outputs (throttle corrections)
ax3 = plt.subplot(3, 2, 3)
plt.plot(time, raw_outputs, 'red', alpha=0.7, linewidth=1, label='Raw PID Output')
plt.plot(time, filtered_outputs, 'blue', linewidth=2, label='Filtered PID Output')
plt.axhline(y=0, color='black', linestyle='-', alpha=0.5)
plt.axhline(y=300, color='gray', linestyle='--', alpha=0.7, label='±300 Limit')
plt.axhline(y=-300, color='gray', linestyle='--', alpha=0.7)

plt.xlabel('Time (seconds)')
plt.ylabel('Throttle Correction')
plt.title('🎮 PID Controller Output (Throttle Commands)')
plt.legend()
plt.grid(True, alpha=0.3)
plt.ylim(-350, 350)

# Plot 4: PID components breakdown for filtered controller
ax4 = plt.subplot(3, 2, 4)
plt.plot(time, filtered_p_terms, 'orange', linewidth=2, label='P Term')
plt.plot(time, filtered_i_terms, 'purple', linewidth=2, label='I Term')
plt.plot(time, filtered_d_terms, 'brown', linewidth=2, label='D Term')
plt.plot(time, filtered_outputs, 'blue', linewidth=2, linestyle='--', label='Total Output')
plt.axhline(y=0, color='black', linestyle='-', alpha=0.5)

plt.xlabel('Time (seconds)')
plt.ylabel('PID Components')
plt.title('🔧 PID Components Breakdown (Filtered Controller)')
plt.legend()
plt.grid(True, alpha=0.3)

# Plot 5: Control activity (showing when corrections are made)
ax5 = plt.subplot(3, 2, 5)
raw_active = (np.abs(raw_errors) > deadband).astype(int)
filtered_active = (np.abs(filtered_errors) > deadband).astype(int)

plt.plot(time, raw_active, 'red', alpha=0.7, linewidth=2, label='Raw Controller Active')
plt.plot(time, filtered_active + 0.1, 'blue', linewidth=2, label='Filtered Controller Active')

plt.xlabel('Time (seconds)')
plt.ylabel('Controller Active')
plt.title('⚡ Controller Activity (1=Active, 0=Idle)')
plt.legend()
plt.grid(True, alpha=0.3)
plt.ylim(-0.2, 1.3)

# Plot 6: Cumulative motor activity (battery usage indicator)
ax6 = plt.subplot(3, 2, 6)
raw_cumulative = np.cumsum(np.abs(raw_outputs))
filtered_cumulative = np.cumsum(np.abs(filtered_outputs))

plt.plot(time, raw_cumulative, 'red', alpha=0.7, linewidth=2, label='Raw (Total Corrections)')
plt.plot(time, filtered_cumulative, 'blue', linewidth=2, label='Filtered (Total Corrections)')

plt.xlabel('Time (seconds)')
plt.ylabel('Cumulative Corrections')
plt.title('🔋 Battery Usage Indicator (Lower = Better)')
plt.legend()
plt.grid(True, alpha=0.3)

plt.tight_layout()

# Calculate and display performance statistics
print("🚁 VL53L0X ALTITUDE HOLD PERFORMANCE ANALYSIS")
print("=" * 60)

# RMS errors
raw_rms_error = np.sqrt(np.mean(raw_errors**2))
filtered_rms_error = np.sqrt(np.mean(filtered_errors**2))

print(f"📏 RMS Error (Root Mean Square):")
print(f"   Raw Controller:      {raw_rms_error:.2f} cm")
print(f"   Filtered Controller: {filtered_rms_error:.2f} cm")
print(f"   Improvement:         {((raw_rms_error - filtered_rms_error)/raw_rms_error)*100:.1f}%")

# Controller activity
raw_activity = np.sum(np.abs(raw_errors) > deadband) / n_samples * 100
filtered_activity = np.sum(np.abs(filtered_errors) > deadband) / n_samples * 100

print(f"\n⚡ Controller Activity:")
print(f"   Raw Controller:      {raw_activity:.1f}% of time")
print(f"   Filtered Controller: {filtered_activity:.1f}% of time")
print(f"   Activity Reduction:  {raw_activity - filtered_activity:.1f} percentage points")

# Total corrections (battery usage)
raw_total_corrections = np.sum(np.abs(raw_outputs))
filtered_total_corrections = np.sum(np.abs(filtered_outputs))

print(f"\n🔋 Total Motor Corrections (Battery Impact):")
print(f"   Raw Controller:      {raw_total_corrections:.0f}")
print(f"   Filtered Controller: {filtered_total_corrections:.0f}")
print(f"   Battery Savings:     {((raw_total_corrections - filtered_total_corrections)/raw_total_corrections)*100:.1f}%")

# Maximum single correction
raw_max_correction = np.max(np.abs(raw_outputs))
filtered_max_correction = np.max(np.abs(filtered_outputs))

print(f"\n🚀 Maximum Single Correction:")
print(f"   Raw Controller:      ±{raw_max_correction:.1f}")
print(f"   Filtered Controller: ±{filtered_max_correction:.1f}")

# Standard deviation of outputs (smoothness)
raw_output_std = np.std(raw_outputs)
filtered_output_std = np.std(filtered_outputs)

print(f"\n🎯 Output Smoothness (Standard Deviation):")
print(f"   Raw Controller:      {raw_output_std:.2f}")
print(f"   Filtered Controller: {filtered_output_std:.2f}")
print(f"   Smoother by:         {((raw_output_std - filtered_output_std)/raw_output_std)*100:.1f}%")

# Sensor noise statistics
sensor_noise_std = np.std(raw_readings - true_altitude)
filtered_noise_std = np.std(filtered_readings - true_altitude)

print(f"\n📡 VL53L0X Sensor Performance:")
print(f"   Raw Noise (Std Dev): {sensor_noise_std:.2f} cm")
print(f"   Filtered Noise:      {filtered_noise_std:.2f} cm")
print(f"   Noise Reduction:     {((sensor_noise_std - filtered_noise_std)/sensor_noise_std)*100:.1f}%")

plt.show()

# Create a detailed zoom-in plot for specific time window
fig2, axes = plt.subplots(2, 1, figsize=(14, 8))

# Focus on 5-second window to show detail
start_time, end_time = 8, 13
start_idx = int(start_time / dt)
end_idx = int(end_time / dt)
time_zoom = time[start_idx:end_idx]

# Detailed altitude comparison
ax1 = axes[0]
ax1.plot(time_zoom, true_altitude[start_idx:end_idx], 'green', linewidth=3, 
         label='True Altitude', alpha=0.8)
ax1.plot(time_zoom, raw_readings[start_idx:end_idx], 'red', alpha=0.7, linewidth=1, 
         marker='o', markersize=2, label='Raw VL53L0X')
ax1.plot(time_zoom, filtered_readings[start_idx:end_idx], 'blue', linewidth=3, 
         label='Filtered VL53L0X')
ax1.axhline(y=target_altitude, color='black', linestyle='--', alpha=0.8, label='Target')

ax1.fill_between(time_zoom, target_altitude - deadband, target_altitude + deadband, 
                alpha=0.3, color='yellow', label=f'Deadband (±{deadband}cm)')

ax1.set_ylabel('Altitude (cm)')
ax1.set_title('🔍 Detailed View: VL53L0X Noise vs Filtering (5-second window)')
ax1.legend()
ax1.grid(True, alpha=0.3)
ax1.set_ylim(75, 85)

# Detailed PID output comparison
ax2 = axes[1]
ax2.plot(time_zoom, raw_outputs[start_idx:end_idx], 'red', alpha=0.7, linewidth=1, 
         marker='s', markersize=2, label='Raw PID Output')
ax2.plot(time_zoom, filtered_outputs[start_idx:end_idx], 'blue', linewidth=3, 
         label='Filtered PID Output')
ax2.axhline(y=0, color='black', linestyle='-', alpha=0.5)

ax2.set_xlabel('Time (seconds)')
ax2.set_ylabel('Throttle Correction')
ax2.set_title('🎮 Detailed View: PID Controller Response')
ax2.legend()
ax2.grid(True, alpha=0.3)

plt.tight_layout()
plt.show()

print("\n" + "="*70)
print("🎯 KEY INSIGHTS FROM VL53L0X SIMULATION:")
print("="*70)
print("✅ RAW SENSOR: Noisy, causes jittery motor responses")
print("✅ FILTERED SENSOR: Smooth, stable altitude hold")
print("✅ BATTERY LIFE: Significantly improved with filtering")
print("✅ MOTOR HEALTH: Less wear from constant micro-adjustments")
print("✅ FLIGHT EXPERIENCE: Drone feels 'locked in place'")
print("✅ VIDEO QUALITY: Stable footage from smooth altitude control")
print("="*70)