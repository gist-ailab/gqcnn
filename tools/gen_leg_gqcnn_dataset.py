"""
Script for converting a tensor dataset to a legacy GQCNN dataset.
Author: Vishal Satish
"""
import logging
import numpy as np
import os
import time
import IPython

import matplotlib.pyplot as plt

from dexnet.learning import TensorDataset
from perception import DepthImage

IM_DATASET_PATH = '/nfs/diskstation/projects/dex-net/parallel_jaws/datasets/dexnet_4/salt_cube/images/'
GRASP_DATASET_PATH = '/nfs/diskstation/projects/dex-net/parallel_jaws/datasets/dexnet_4/salt_cube/grasps/'
CROP_SIZE = 46
NUM_CHANNELS = 1
IM_PER_FILE = 100
POSE_DIM = 6
# GQCNN_SCALE_FACTOR = 0.5
GQCNN_SCALE_FACTOR = 1.0
FNAME_PLACE = 6
IM_FILE_TEMPLATE = 'depth_ims_tf_table'
POSE_FILE_TEMPLATE = 'hand_poses'
METRIC_FILE_TEMPLATE = 'robust_wrench_resistance'
OUTPUT_DIR = '/nfs/diskstation/vsatish/dex-net/data/datasets/salt_cube_leg_no_rot_04_23_18/'
ROTATE = 0
NUM_RANDOM_DEPTHS = 0
GRIPPER_DEPTH = 0.07
METRIC_THRESH = 0.75
VIS = 0
DEBUG = 0

if __name__ == '__main__':
    # setup logger
    logging.getLogger().setLevel(logging.INFO)    

    # get start time
    gen_start_time = time.time()

    # open datasets
    logging.info('Opening datasets')
    im_dataset = TensorDataset.open(IM_DATASET_PATH)
    grasp_dataset = TensorDataset.open(GRASP_DATASET_PATH)
    
    # generate buffers
    logging.info('Allocating buffers')
    im_buffer = np.zeros((IM_PER_FILE + NUM_RANDOM_DEPTHS, CROP_SIZE, CROP_SIZE, NUM_CHANNELS))
    pose_buffer = np.zeros((IM_PER_FILE + NUM_RANDOM_DEPTHS, POSE_DIM))
    metric_buffer = np.zeros((IM_PER_FILE + NUM_RANDOM_DEPTHS,))

    # iterate through the image dataset
    buffer_ind = 0
    out_file_idx = 0
    for im_idx, datum in enumerate(im_dataset):
        im = DepthImage(datum['depth_ims'])
        grasp_start_ind = datum['grasp_start_ind']
        grasp_end_ind = datum['grasp_end_ind']
        
        # iterate through the corresponding grasps
        for i in range(grasp_start_ind, grasp_end_ind + 1):
            grasp = grasp_dataset[i]
            metric = grasp['grasp_metrics']
            pose = grasp['grasps']            

            # align grasp
            logging.info('Aligning grasp {} of {} for image {} of {}'.format(i - grasp_start_ind, grasp_end_ind - grasp_start_ind, im_idx, im_dataset.num_datapoints))
            
            # rotate if training a normal GQCNN, else don't rotate for angular GQCNN
            if ROTATE:
                rot_ang = pose[3]
            else:
                rot_ang = 0

            # center, crop, rotate, and rescale
            tf_im = im.align(GQCNN_SCALE_FACTOR, np.asarray([pose[1], pose[0]]), rot_ang, CROP_SIZE, CROP_SIZE)

            # vis original bin image and crop
            if VIS:
                logging.info('Crop X: {}, Crop Y: {}'.format(pose[1], pose[0]))
                plt.figure()
                plt.subplot(121)
                plt.imshow(im.raw_data[..., 0], cmap=plt.cm.gray)
                plt.subplot(122)
                plt.imshow(tf_im.raw_data[..., 0], cmap=plt.cm.gray)
                plt.show()
 
            # generate random grasps depth-wise
            if NUM_RANDOM_DEPTHS > 0 and metric > METRIC_THRESH:
                logging.info('Generating random grasps...')
                exc_range_low = tf_im.raw_data[CROP_SIZE // 2, CROP_SIZE // 2, 0]
                exc_range_high = exc_range_low + GRIPPER_DEPTH
                new_pose = np.copy(pose)
                for i in range(NUM_RANDOM_DEPTHS):
                    logging.info('Generating random grasp {} of {}'.format(i, NUM_RANDOM_DEPTHS))
                    rand_depth = np.random.random()
                    while exc_range_low < rand_depth < exc_range_high:
                          rand_depth = np.random.random()
                    if DEBUG:
                        logging.info('Sampled random depth: {}'.format(rand_depth))
                    # generate new pose with random depth
                    new_pose[2] = rand_depth                    

                    # add to buffers
                    im_buffer[buffer_ind, ...] = tf_im.raw_data
                    pose_buffer[buffer_ind, ...] = new_pose
                    metric_buffer[buffer_ind, ...] = 0.0
                    buffer_ind += 1
                     
            # add to buffers
            im_buffer[buffer_ind, ...] = tf_im.raw_data
            pose_buffer[buffer_ind, ...] = pose
            metric_buffer[buffer_ind, ...] = metric
            buffer_ind += 1            

            # write out when buffers are full
            if buffer_ind >= IM_PER_FILE:
                # dump IM_PER_FILE datums
                logging.info('Saving {} datapoints'.format(IM_PER_FILE))
                im_fname = '{}_{}'.format(IM_FILE_TEMPLATE, str(out_file_idx).zfill(FNAME_PLACE))
                pose_fname = '{}_{}'.format(POSE_FILE_TEMPLATE, str(out_file_idx).zfill(FNAME_PLACE))
                metric_fname = '{}_{}'.format(METRIC_FILE_TEMPLATE, str(out_file_idx).zfill(FNAME_PLACE))
                np.savez_compressed(os.path.join(OUTPUT_DIR, im_fname), im_buffer[:IM_PER_FILE])
                np.savez_compressed(os.path.join(OUTPUT_DIR, pose_fname), pose_buffer[:IM_PER_FILE])
                np.savez_compressed(os.path.join(OUTPUT_DIR, metric_fname), metric_buffer[:IM_PER_FILE])
                out_file_idx += 1
                im_buffer[:buffer_ind % IM_PER_FILE] = im_buffer[IM_PER_FILE:buffer_ind]
                pose_buffer[:buffer_ind % IM_PER_FILE] = pose_buffer[IM_PER_FILE:buffer_ind]
                metric_buffer[:buffer_ind % IM_PER_FILE] = metric_buffer[IM_PER_FILE:buffer_ind]
                buffer_ind = buffer_ind % IM_PER_FILE
    logging.info('Dataset generation took {} seconds'.format(time.time() - gen_start_time))
