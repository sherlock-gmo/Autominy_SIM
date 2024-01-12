# Autominy_SIM
This repository is a collection of software developed for the [AutoMiny](https://autominy.github.io/AutoMiny/) vehicle. This software runs on simulator: https://github.com/ITAM-Robotica/EK_AutoNOMOS_Sim

## Required Dependencies
This software was tested on ROS melodic and ROS noetic.
- [Gazebo Math](https://gazebosim.org/api/math/7/install.html)
- [Ignition Transport v4] (https://gazebosim.org/api/transport/9.0/installation.html)
- [CUDA 11.7](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html) See for example some of these pages: [1](https://techzizou.com/install-cuda-and-cudnn-on-windows-and-linux/#linux), [2](https://medium.com/geekculture/yolov4-darknet-installation-and-usage-on-your-system-windows-linux-8dec2cea6e81#a59a), [3](https://pjreddie.com/darknet/yolo/)
- cuDNN 8.4.1.50
- OpenCV  Install it as [here](https://efcomputer.net.au/blog/4-steps-to-install-darknet-with-cuda-and-opencv-for-realtime-object-detection/).
- [darnet](https://github.com/leggedrobotics/darknet/tree/d22bbf38bd012f13d2b50c8d98149cd4a9889b7a)
- [dark_net_ros](https://github.com/leggedrobotics/darknet_ros)

## Building
-	Clone the repository:

		git clone https://github.com/dotmex-z/dotMEX_Autominy_SIM
	
-	Create the darknet repository in dotMEX_Autominy_SIM/src 

		cd dotMEX_Autominy_SIM/src
		git clone https://github.com/leggedrobotics/darknet/tree/d22bbf38bd012f13d2b50c8d98149cd4a9889b7a
		cd darknet

-	I recommend to use gpu and opencv configuration. Edit the Makefile to add the [gpu-architecture](https://developer.nvidia.com/cuda-gpus) and change the lines:

		GPU=1
		CUDNN=0
		CUDNN_HALF=0
		OPENCV=1

-	Then use the [make](https://pjreddie.com/darknet/install/) instruction.

-	Add the neural network's weights to the dotMEX_Autominy_SIM/autominy_ws/src/darknet_ros/yolo_network_config/weights folder. Choose the file [dotmex_yolov3_15000.weights](https://drive.google.com/drive/folders/1a95cmAPXt_KvZuGdBtEg6sZWuQqUulx1?usp=sharing) 

-	Also edit the file dotMEX_Autominy_SIM/autominy_ws/src/dotmex2022/scripts/behavior_selector.py and change the "path_libs" variable correctly (line 11).

-	Finally compile the workspace:

		cd dotMEX_Autominy_SIM/src
		catkin_init_workspace
		cd ..
		catkin_make -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=/usr/bin/gcc-8

## Test	
-	Modify the .bashrc file to add the following lines with your correct path:

		export GAZEBO_PLUGIN_PATH=:/home/dotmex/dotMEX_Autominy_SIM/autominy_ws/devel/lib
		export GAZEBO_MODEL_PATH=:/home/dotmex/dotMEX_Autominy_SIM/autominy_ws/src/autonomos_gazebo_simulation/models
		export GAZEBO_RESOURCE_PATH=:/home/dotmex/dotMEX_Autominy_SIM/autominy_ws/src/autonomos_gazebo_simulation/worlds
		source /home/dotmex/dotMEX_Autominy_SIM/autominy_ws/devel/setup.bash

- Charge the world on Gazebo using:

		roslaunch autonomos_gazebo_simulation roscar_city.launch

-	Begin the darnet_ros node. If you want to see detected objects, you can use 'rqt_image_view' and subscribe it to the topic '/darknet_ros/detection_image':

		roslaunch darknet_ros dotmex_darknet_ros.launch
		rqt_image_view /darknet_ros/detection_image
		
- Begin the behavior_selector node. 

		rosrun dotmex2022 behavior_selector.py
		
- Due to the red light, the car won't move. Move the red light out of the car's camera's visual field. Then you'll see how the car lane-keeps while its speed change; depending on the speed limit signal detected. When the car detects another vehicle in its lane, it will begin the passing maneuver; however, a pedestrian will appear and suddenly it'll stop. Then, move Jhon out of the lane and the car will drive lane-keeping. Finally, to park the car, you have to send the following message when it's close to the blue cars' formation:

		rostopic pub -1 /parking std_msgs/Bool "data: true"
		
- Check the [video](https://youtu.be/2t755lCvivU) of simulation.


## Citation
Please, cite us as:
```
@inproceedings{gonzalez2022behavior,
  title={Behavior selector for autonomous vehicles using neural networks},
  author={Gonz{\'a}lez-Miranda, Oscar and Ibarra-Zannatha, Juan Manuel},
  booktitle={2022 XXIV Robotics Mexican Congress (COMRob)},
  pages={31--35},
  year={2022},
  organization={IEEE}
}
```



	

