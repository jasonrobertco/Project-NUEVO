import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Load CSV
path = "dc_motor_1_0402_19-04.csv"
df = pd.read_csv(path)

t = df["Time (s)"].to_numpy()
v = df["Velocity (t/s)"].to_numpy()
pwm = df["PWM"].to_numpy()

# Assumed setpoint from the lab step-response test
setpoint = 1000.0

# Detect start of step from first nonzero PWM
i0 = int(np.argmax(np.abs(pwm) > 0))
t0 = t[i0]

t_rel = t - t0
mask = t_rel >= 0
t_plot = t_rel[mask]
v_plot = v[mask]

def first_crossing(time_arr, value_arr, threshold):
    idx = np.where(value_arr >= threshold)[0]
    if len(idx) == 0:
        return None, None
    i = int(idx[0])
    return i, time_arr[i]

# Rise time
_, t10 = first_crossing(t_plot, v_plot, 0.1 * setpoint)
_, t90 = first_crossing(t_plot, v_plot, 0.9 * setpoint)
rise_time = None if (t10 is None or t90 is None) else (t90 - t10)

# Overshoot
peak_idx = int(np.argmax(v_plot))
peak_val = float(v_plot[peak_idx])
peak_time = float(t_plot[peak_idx])
overshoot_pct = (peak_val - setpoint) / setpoint * 100.0

# Settling time (±5%)
band_low = 0.95 * setpoint
band_high = 1.05 * setpoint
outside = (v_plot < band_low) | (v_plot > band_high)
outside_idx = np.where(outside)[0]
if len(outside_idx) == 0:
    settling_time = 0.0
else:
    last_out = int(outside_idx[-1])
    settling_time = float(t_plot[last_out + 1]) if last_out + 1 < len(t_plot) else float("nan")

# Steady-state error
steady_state_value = float(v_plot[int(len(v_plot) * 0.9):].mean())
steady_state_error = setpoint - steady_state_value

# Plot
fig, ax = plt.subplots(figsize=(12, 7))
ax.plot(t_plot, v_plot, linewidth=2, label="Velocity")
ax.axhline(setpoint, linestyle="--", linewidth=1.5, label="Setpoint")
ax.axhline(band_low, linestyle=":", linewidth=1, label="±5% band")
ax.axhline(band_high, linestyle=":", linewidth=1)

if t10 is not None:
    ax.axvline(t10, linestyle=":", linewidth=1)
    ax.annotate("10%", xy=(t10, 0.1 * setpoint), xytext=(t10 + 0.08, 0.18 * setpoint),
                arrowprops=dict(arrowstyle="->"))
if t90 is not None:
    ax.axvline(t90, linestyle=":", linewidth=1)
    ax.annotate("90%", xy=(t90, 0.9 * setpoint), xytext=(t90 + 0.08, 0.78 * setpoint),
                arrowprops=dict(arrowstyle="->"))
if rise_time is not None:
    ax.annotate(f"Rise time = {rise_time:.3f} s",
                xy=((t10 + t90) / 2, 0.55 * setpoint),
                xytext=(0.55, 0.35 * setpoint),
                arrowprops=dict(arrowstyle="->"))

ax.plot(peak_time, peak_val, marker="o")
ax.annotate(f"Peak = {peak_val:.0f}\nOvershoot = {overshoot_pct:.1f}%",
            xy=(peak_time, peak_val),
            xytext=(peak_time + 0.2, peak_val - 200),
            arrowprops=dict(arrowstyle="->"))

if np.isfinite(settling_time):
    ax.axvline(settling_time, linestyle="--", linewidth=1)
    ax.annotate(f"Settling time = {settling_time:.3f} s",
                xy=(settling_time, setpoint),
                xytext=(settling_time + 0.2, setpoint + 180),
                arrowprops=dict(arrowstyle="->"))

ax.annotate(f"Steady-state error = {steady_state_error:.2f}",
            xy=(t_plot[-1], steady_state_value),
            xytext=(max(t_plot[-1] - 1.0, 0.2), steady_state_value + 180),
            arrowprops=dict(arrowstyle="->"))

ax.set_title("DC Motor Step Response with Task 4 Metrics")
ax.set_xlabel("Time since step command (s)")
ax.set_ylabel("Velocity (ticks/s)")
ax.legend()
ax.grid(True)
plt.tight_layout()
plt.show()

print(f"Rise time tr: {rise_time:.3f} s" if rise_time is not None else "Rise time tr: not found")
print(f"Percent overshoot Mp: {overshoot_pct:.1f}%")
print(f"Settling time ts: {settling_time:.3f} s" if np.isfinite(settling_time) else "Settling time ts: not found")
print(f"Steady-state error ess: {steady_state_error:.2f} ticks/s")