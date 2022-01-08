# time_sync_bag_to_csv package

This package converts .bag collected by asv (but any type of robot) into .csv.
The robot is assumed to record .bag file including gps, imu, compass, depth sonar, ysi sonde, ardupilot (mavros). 
* reference: http://wiki.ros.org/message_filters/ApproximateTime

## License
This project is licensed under the MIT License.

Authors: Mingi Jeong, Monika Roznere (contribution to gds tool)

## Prerequisites
You need to have ROS (equal to or above kinetic), Ubuntu (equal to or above 16.04).


### Dependencies

1. ysi_exo

    Please install the following package beforehand.
    ```
    https://github.com/dartmouthrobotics/ysi_exo
    ```
2. mavros
    ```
    sudo apt-get install ros-yourversion-mavros ros-yourversions-mavros-extras
    ```

### Note
1. Note that the ROS bag file records several “topics” with different “message types” for the equipped sensors. 
2. Each topic has a different frequency of signals such that the recording frequency is different on a .bag file.
3. The goal is to make synchronization of data, particularly, with respect to the sonde data recorded at every 1 sec. 

### Data preparation

1. The `sonde data scheme must be compliant` with the consistent format we’ve been using. One of the most common mistakes was to forget to change back to the ROS logging format after hand deployments. Please look at the following link: https://docs.google.com/document/d/1tNG-p9tAx_645k2QzPeLvTCgX4bAZtucSeQsj7cPvrQ/edit?usp=sharing

2. Recording must be terminated cleanly with the following commands: https://docs.google.com/document/d/1R8O4KQyuBIzqikTjKqOOcuMz0GErfZbymMTKQuj2oPc/edit#bookmark=id.n2rfw04h8hj4
Do not just change the name .active file (terminated unexpectedly) to .bag file. If it was terminated as .active, please upload as it is, so that we can recover (if possible).

3. The sonde must be time-synchronized with a laptop. 
    * Make sure that the laptop displays the correct time compared to real-time clock on the internet. https://time.is/
    * Make sure that on KOREXO, the time synchronization activation button is clicked such that whenever sonde is connected before deployment, the time synchronization is executed for proper logging of ROSbag. 


## Install
Clone this package into your `catkin_ws` and do `catkin_make` for building.

```
cd catkin_workspace/src
git clone https://github.com/dartmouthrobotics/time_sync_bag_to_csv.git
cd ..
catkin_make
```

## Usage

### 1. configuration of .yaml file
This param.yaml file sets up 
* output .csv file name
* chosen sensor data as desired output
* chosen sensor topic name (e.g., sonar: /depth_transducer/depth)

### 2. Running 
* replay the bag file
    ```
    rosbag play ~.bag
    ```
* run the converter node
    ```
    roslaunch time_sync_bag_to_csv converter.launch
    ```
    * Once it runs corretly, you will see the printed msg that it is conversting.
    * If the conversion is finished, press `ctrl+c` and go to the output folder

### TODO
* header naming
* GUI
* Marvros WP only


## Citation
If you use this package, please cite the following paper by Rozenere and Jeong et al 2021 for ISER (International Symposium on Experimental Robotics).
```
@inproceedings{iser2021,
author = {Roznere, Monika and Jeong, Mingi and Maechling, Lily and Ward, Nicole K and Brentrup, Jennifer A and Steele, Bethel and Bruesewitz, Denise A and Ewing, Holly A and Weathers, Kathleen C and Cottingham, Kathryn L and {Quattrini Li}, Alberto},
booktitle = {Experimental Robotics},
isbn = {978-3-030-71151-1},
pages = {139--150},
publisher = {Springer International Publishing},
title = {{Towards a Reliable Heterogeneous Robotic Water Quality Monitoring System: An Experimental Analysis}},
year = {2021}
}

```

