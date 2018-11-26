# Mask R-CNN for Object Detection and Segmentation
This is an implementation of [Mask R-CNN](https://arxiv.org/abs/1703.06870) on Python 3, Keras, and TensorFlow. The model generates bounding boxes and segmentation masks for each instance of an object in the image. It's based on Feature Pyramid Network (FPN) and a ResNet101 backbone.

The repository includes:
* Source code of Mask R-CNN built on FPN and ResNet101.
* Fully-trained weights for grasp detection
* Jupyter notebook for grasp detection
* Jupyter notebooks to visualize the detection pipeline at every step

# Code Tested on 
Ubuntu 16.04 LTS
Anaconda 4.5.11
Jupyter Notebook 5.7.0

## Requirements for MaskRCNN
numpy
scipy
Pillow
cython
matplotlib
scikit-image
tensorflow>=1.3.0
keras>=2.0.8
opencv-python
h5py
imgaug
IPython[all]

## Requirements for ROS
ROS Kinetic 
universal_robot package
ur_modern_driver package 
robotiq package
MoveIt! 

# Getting Started
* ([model.py](mrcnn/model.py), [utils.py](mrcnn/utils.py), [config.py](mrcnn/config.py)): These files contain the main Mask RCNN implementation. 

* [inspect_data.ipynb](samples/project/inspect_data.ipynb). This notebook visualizes the different pre-processing steps
to prepare the training data.

* [inspect_model.ipynb](samples/project/inspect_model.ipynb) This notebook goes in depth into the steps performed to detect and segment objects. It provides visualizations of every step of the pipeline.

* [object_localizer.ipynb](samples/project/object_localizer.ipynb) This notebook performs object detection of gripper, cubes, and cylinders and publishes relevent pose data as a ROS topic named /chatter. 

* Note: This project runs anaconda and ROS together. Having an active Anaconda path in your .bashrc will cause errors when you try to use ROS. To resolve this conflict, refer to (http://wiki.ros.org/IDEs).  
You may or may not have issues importing ros/robotiq packages or cv2 when you run object_localizer.ipynb. Depending on the error message, remove appropriate system path using line 39: 
```
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
```
To run only the Mask RCNN without ROS, comment out the ROS related imports in [object_localizer.ipynb]. 

# Training 
Pre-trained weights for MS COCO has been provided to make it easier to start. 
You can use those weights as a starting point to train your own variation of the network. 
Training and evaluation code is in `samples/project/project.py`. 
You can import this module in Jupyter notebook or you can run it directly from the command line as such:

```
# Train a new model starting from pre-trained COCO weights
python3 samples/project/project.py train --dataset=/path/to/dataset/ --model=coco

# Continue training the last model you trained. This will find
# the last trained weights in the model directory.
python3 samples/project/project.py train --dataset=/path/to/dataset/ --model=last
```

The training schedule, learning rate, and other parameters should be set in `samples/project/project.py`.
The code in `project.py` is set to train for 3K steps (30 epochs of 100 steps each), and using a batch size of 1. 
Update the schedule to fit your needs.

# Credits
Matterport, Inc.
github: https://github.com/matterport/Mask_RCNN
Refer to LICENSE. 
