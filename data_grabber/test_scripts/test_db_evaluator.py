import pickle
import matplotlib.pyplot as plt
import numpy as np

'''
file_name = 'data/test/object_III.pickle'

with open(file_name, 'rb') as f:
    db = pickle.load(f)

initial_image = db['II']['pressing'][0]['initial_image']
rgb_image = initial_image['rgb_image']
plt.imshow(rgb_image)
plt.show()

fsr_data = np.array(db['III']['pressing'][0]['fsr_data'])
action_data = np.array(db['III']['pressing'][0]['action_data'])
ft_data = np.array(db['III']['pressing'][0]['ft_data'])

mean_fsr = np.mean(fsr_data, axis=1)
plt.plot(mean_fsr, 'r')
plt.plot(ft_data[:, 2], 'b')
plt.show()
'''
with open('test.npy', 'rb') as f:
    a = np.load(f)
color = ['r', 'g', 'b', 'k', 'm', 'c']
ind = 0
for i in range(0, 500, 100):
    print(i)
    plt.plot(a[i, :], color[ind])
    ind += 1

plt.show()