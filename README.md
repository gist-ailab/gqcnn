# Berkeley AUTOLAB's GQCNN Package
<p>
   <a href="https://travis-ci.org/BerkeleyAutomation/gqcnn/">
       <img alt="Build Status" src="https://travis-ci.org/BerkeleyAutomation/gqcnn.svg?branch=master">
   </a>
   <a href="https://github.com/BerkeleyAutomation/gqcnn/releases/latest">
       <img alt="Release" src="https://img.shields.io/github/release/BerkeleyAutomation/gqcnn.svg?style=flat">
   </a>
   <a href="https://github.com/BerkeleyAutomation/gqcnn/blob/master/LICENSE">
       <img alt="Software License" src="https://img.shields.io/badge/license-REGENTS-brightgreen.svg">
   </a>
   <a>
       <img alt="Python 3 Versions" src="https://img.shields.io/badge/python-3.5%20%7C%203.6%20%7C%203.7-yellow.svg">
   </a>
</p>

## Package Overview
The gqcnn Python package is for training and analysis of Grasp Quality Convolutional Neural Networks (GQ-CNNs). It is part of the ongoing [Dexterity-Network (Dex-Net)](https://berkeleyautomation.github.io/dex-net/) project created and maintained by the [AUTOLAB](https://autolab.berkeley.edu) at UC Berkeley.

## Installation and Usage
Please see the [docs](https://berkeleyautomation.github.io/gqcnn/) for installation and usage instructions.

## Citation
If you use any part of this code in a publication, please cite [the appropriate Dex-Net publication](https://berkeleyautomation.github.io/gqcnn/index.html#academic-use).


## ROS

model_name:=GQCNN-4.0-PJ or model_name:=FC-GQCNN-4.0-PJ:
```
$ ROS_NAMESPACE="wr" roslaunch unstructured_task_manager rs_d435_wrist.launch
$ roslaunch gqcnn grasp_planning_service.launch model_name:=FC-GQCNN-4.0-PJ
$ rosrun gqcnn grasp_planner_requester.py
$ rosservice call /gqcnn/grasp_planner_request
```

## Notes

Qt Error: This application failed to start because no Qt platform plugin could be initialized. Reinstalling the application may fix this problem.

``
pip install opencv-python==4.1.2.30  
```