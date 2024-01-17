How to launch demo: 

In every terminal : source install/local_setup.bash

launch file
type on terminal1 : ros2 launch rover_lane lane_sub.launch.py

enable service
type on terminal2 : ros2 service call /rover/lane_sub/enable std_srvs/srv/SetBool "{data: true}"

launch ros2 image publisher
type on terminal2 : ros2 run cv_basics img_publisher


Lancio degli aruco detector: 

 
Terminale 1 lanciare: “ros2 launch ros2_examples_bringup image_processing_pipeline.launch.py” 

Terminale 2 : “ros2 run rqt_image_view rqt_image_view &” apro visualizzatore 

Terminale 2 : eseguire “ros2 service call /image_processing_pipeline/usb_camera_driver/enable_camera std_srvs/srv/SetBool "{data: true}"” 

Terminale 3 eseguire : ros2 service call /image_processing_pipeline/aruco_detector/enable std_srvs/srv/SetBool "{data: True}" 


For launch node lane_sub in real world change in rover_lane_sub.cpp on line 48:
topic -- "/car1/usb_camera_driver/camera/image_rect_color" for real Use
topic -- "video_frames" for demo with webcam_pub publisher


// LANCIO REALSENSE PER NODO depth

ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true



// Parameter realsense camera
Parameter name: depth_module.min_distance
  Type: integer
  Description: Minimal distance to the target (in mm)
Default value: 490
  Constraints:
    Min value: 0
    Max value: 8000
    Step: 1

Parameter name: .depth.image_rect_raw.tiff.res_unit
  Type: string
  Description: tiff resolution unit
  Constraints:
    Additional constraints: Supported values: [none, inch, centimeter]


