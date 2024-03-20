import matplotlib.pyplot as plt
import pickle
import numpy as np
import os
import re
# import imageio

scans = 14
object_id_map = {'I': 1, 'II': 2, 'III': 3, 'IV': 4, 'V': 5, 'VI': 6, 'VII': 7, 'VIII': 8, 'IX': 9, 'X': 10}
traj_keys = ['initial_image', 'params', 'fsr_data', 'action_data', 'ft_data']
exploratory_actions = ['pressing', 'precision']
min_value = -70
max_value = 560
viz_directory = 'viz/'


def process_data(file_name, object_id):
    global_max = 0
    global_min = 0
    db = pickle.load(file_name)

    for exp_action in exploratory_actions:
        folder_name = object_id + '_' + str(exp_action)
        path = os.path.join(viz_directory, folder_name)
        if not os.path.exists(path):
            os.mkdir(path)
        for scan in range(0, scans):
            initial_image_data = db[object_id][exp_action][scan]['initial_image']
            rgb_image = initial_image_data['rgb_image']
            depth_image = initial_image_data['depth_image']
            traj_param = db[object_id][exp_action][scan]['params']
            baseline = traj_param['baseline']
            baseline = baseline.reshape(16, 16)

            tactile_obs = db[object_id][exp_action][scan]['fsr_data']
            action_obs = db[object_id][exp_action][scan]['action_data']
            ft_obs = db[object_id][exp_action][scan]['ft_data']

            # Generate plots for each scan
            # Save the initial RGB-D image
            fig, ax = plt.subplots(2, 3)
            ax[0, 0].imshow(rgb_image)
            ax[0, 1].imshow(depth_image)

            tactile_obs = np.array(tactile_obs)
            mean_tactile = np.mean(tactile_obs, axis=1)
            max_tactile = np.max(tactile_obs)
            min_tactile = np.min(tactile_obs)

            if max_tactile > global_max:
                global_max = max_tactile

            if min_tactile < global_min:
                global_min = min_tactile

            # print('exp_action - ', exp_action, ' scan - ', scan, ' Max_val - ', np.max(tactile_obs))

            # A GIF for the tactile_obs
            # rescale
            tactile_obs_rescaled = ((tactile_obs - min_value) * (1 / (max_value - min_value) * 255)).astype('uint8')

            tactile_obs_vid = tactile_obs_rescaled.reshape(549, 16, 16)
            tactile_obs_vid = tactile_obs_vid.astype(np.uint8)
            gif_file_name = path + '/scan-'+str(scan)+ '.gif'
            # imageio.mimsave(gif_file_name, tactile_obs_vid, fps=50)

            # Mean Tactile plot + FT plot and Action plot (proprioception)
            ft_obs = np.array(ft_obs)
            action_obs = np.array(action_obs)
            ax[1, 0].plot(ft_obs[:, 2], 'r')
            ax[1, 0].plot(mean_tactile, 'b')
            ax[1, 1].plot(action_obs[:, 2], 'r', linewidth=2)
            ax[0, 2].imshow(baseline)
            ax[1, 2].plot(action_obs[:, 4], 'g')

            figure_filename = path + '/scan-'+str(scan)+ '.png'
            plt.savefig(figure_filename)
            plt.close()
    return global_max, global_min


data_dir = 'data/soft_object/'
files = os.listdir(data_dir)
files.sort()

for file in files:
    print('Currently processing-', file)
    file_name = os.path.join(data_dir, file)
    data_file = open(file_name, 'rb')
    file_name_splits = re.split('_|\.', file)
    object_id = file_name_splits[1]
    global_max_val, global_min_val = process_data(data_file, object_id)
    print("Max Tactile Value: ", global_max_val, " Min Tactile Value: ", global_min_val)

    data_file.close()

