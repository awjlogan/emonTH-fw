"""Calculate intensity array for LED pulse effect at startup"""

T_TIMER = 8e-6
N_STEPS = 32

top_val = 1 / (N_STEPS * 2) / T_TIMER

# Approach maximum value as square
scaleSqr = top_val / N_STEPS**2

intensities = [int(scaleSqr * i**2) for i in range(1, N_STEPS + 1)]

# Generate array
print(f"static const uint16_t ledIntensity[{N_STEPS}] = {{", end="")
for intensity in intensities[:-1]:
    print(f"{intensity}, ", end="")

print(f"{intensities[-1]}}};")
