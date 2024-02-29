// LANCIO REALSENSE PER NODO - Lancia camera

ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true

----------------------------------------------------------------------
How to launch lane assist: 

In every terminal : source install/local_setup.bash

launch file
type on terminal1 : ros2 launch rover_lane lane_sub.launch.py

enable service
type on terminal2 : ros2 service call /rover/lane_sub/enable std_srvs/srv/SetBool "{data: true}"

optional - launch ros2 image publisher only for a demo with video file - change subsriber topic in rover_lane_sub.cpp
type on terminal2 : ros2 run cv_basics img_publisher

For launch node lane_sub in real world change in rover_lane_sub.cpp on line 54:
topic -- "/camera/camera/color/image_raw" for real Use
topic -- "video_frames" for demo with webcam_pub publisher

-----------------------------------------------------------------------

How to launch obstacle detection:

ros2 run obstacle_detection obstacle

-----------------------------------------------------------------------

// Parameter realsense camera
Parameter name: depth_module.min_distance
  Type: integer
  Description: Minimal distance to the target (in mm)
Default value: 490
  Constraints:
    Min value: 0
    Max value: 8000
    Step: 1

-------------------------------------------------------------------------

How to install Realsense driver

https://github.com/IntelRealSense/realsense-ros  -- setup guide link

Part 1 - Install Realsense SDK 2.0
          from https://github.com/IntelRealSense/librealsense/blob/master/doc/installation_jetson.md

          - sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
          - sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
          - sudo apt-get install librealsense2-utils
          - sudo apt-get install librealsense2-dev

Part 2 - Install Ros2 Wrapper

          from source 

          create ros2 Workspace : 

          mkdir -p ~/ros2_ws/src
          cd ~/ros2_ws/src/

          clone: 

          git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-development
          cd ~/ros2_ws

          install dependencies

          sudo apt-get install python3-rosdep -y
          sudo rosdep init # "sudo rosdep init --include-eol-distros" for Foxy and earlier
          rosdep update # "sudo rosdep update --include-eol-distros" for Foxy and earlier
          rosdep install -i --from-path src --rosdistro $ROS_DISTRO --skip-keys=librealsense2 -y

          build with colcon build

          source enviroment.

NOTE!! - Remove camera id control from source file in realsense2_camera/src/realsense_node_factory.cpp

        line 379.

