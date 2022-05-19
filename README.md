# camera_drone_calibration

A toolbox for solving the spatial calibration of an monocular camera w.r.t a object body frame in motion capture system along with camera intrinsic parameters

## Prerequisites

Most of the dependencies are standard numpy, scipy is dependent on Python 3.8 and ROS Noetic/Melodic. Please ensure Python 3.8 as other python versions may lead to build and import errors.

```
sudo apt-get install libsuitesparse-dev
```

## Compiling

```
$ cd /path/to/your/workspace/src
$ git clone --branch master https://github.com/gred0216/camera_drone_calibration.git
$ cd ..
$ catkin build camera_drone_calibration
$ source /path/to/your/workspace/devel/setup.bash
```


## Running

1. Put the calibration board on the ground. The text on the board should face front. [TODO: pic]
3. Fix the camera facing the calibration board. Camera should face the baord as perpendicular as possible.
4. `$ roslaunch camera_drone_calibration camera_drone_calibration.launch`
5. Collect the data for 5 seconds when camera is still. Red grid points in world frame would be re-projected to camera frame. You can move the camera around to see the effect.
6. Result will be saved to yaml file in /result/result.yaml

The table below summarized the default publication and subscription scheme. The subsciption topic names can be changed in the launch file.

### Subsciption
|Name|Type|Description|
|---|---|---|
|`/camera/color/image_raw`|Image||
|`/vicon/race1/odom`|Odom||

### Publication
|Name|Type|Description|
|---|---|---|
|`camera/color/image_raw_feature`|Image||
|`/camera/body/pose`|PoseStamped||



### Demo Video

video



## References
The calibration approaches used in Camera Drone Calibration are based on the following papers. Please cite the appropriate papers when using this toolbox or parts of it in an academic publication.

1. Seong Hun Lee, Javier Civera (2020). Robust single rotation averaging. arXiv preprint arXiv:2004.00732

## TODO
1. color channel
2. distortion model
