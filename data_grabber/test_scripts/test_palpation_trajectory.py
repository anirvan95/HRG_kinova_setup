import numpy as np
import matplotlib.pyplot as plt

'''
# For precession motion
sampling_freq = 50  # Hz
total_time = 20  # seconds total time of action, fixed for now
time_index = np.linspace(0, total_time, sampling_freq*total_time)

quarter_time = 5  # seconds # intuitive definition, please don't judge
palpation_frequency = 1.0/(quarter_time*4.0)
max_amplitude = 15  # max movement angle

theta_trajectory = max_amplitude*np.sin(2*np.pi*palpation_frequency*time_index)
omega_trajectory = 2*np.pi*palpation_frequency*max_amplitude*np.cos(2*np.pi*palpation_frequency*time_index)
plt.plot(time_index, theta_trajectory, 'r')
plt.plot(time_index, omega_trajectory, 'b')
plt.show()
'''
# For pressing motion
sampling_freq = 50  # Hz
total_time = 1  # seconds total time of action, fixed for now
time_index = np.linspace(0, total_time, sampling_freq*total_time)

quarter_time = 1 # seconds # intuitive definition, please don't judge
palpation_frequency = 1.0/(quarter_time*4.0)
max_depth = 0.003  # 3 mm palpation

z_trajectory = max_depth*np.sin(2*np.pi*palpation_frequency*time_index)
vz_trajectory = 2*np.pi*palpation_frequency*max_depth*np.cos(2*np.pi*palpation_frequency*time_index)
vz_clipped = []
clip_time = 0.0  # time in seconds
delta = quarter_time - clip_time
for t in range(0, len(vz_trajectory)):
    if clip_time <= t < quarter_time + delta:
        vz_clipped.append(0)
    else:
        vz_clipped.append(vz_trajectory[t])

plt.plot(vz_trajectory, 'r')
plt.plot(np.array(vz_clipped), 'b')
plt.show()



