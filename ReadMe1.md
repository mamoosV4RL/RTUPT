Readme:


Basic Usage
-----------

The Pose Estimator makes use of a number of parameters to estimate the pose of the tracked object. These include the location of the LEDs on the object, the intrinsic parameters of the camera, and various runtime parameters, such as thresholding values.

#### Setting up the Marker Positions Parameter File

In order to predict the pose of the target object, the Pose Estimator needs to know the positions of the LEDs on the object in the object's frame of reference. These positions are given to the program using a YAML file located within the `roscd monocular_pose_estimator/marker_positions` folder.

The file is called `nameOfTheMarkerPositionFile.YAML` and has the following format.

    # The marker positions in the trackable's frame of reference:
    #
    marker_positions:
  	- x:  0.000  
  	  y:  0.000 
  	  z:  0.000
  	- x:  0.004
  	  y: -0.188
  	  z:  0.039
  	- x:  0.228
  	  y: -0.140
  	  z:  0.000
  	- x:  0.264
  	  y:  0.124
 	  z:  0.000
  	- x:  0.076
  	  y:  0.128
  	  z:  0.005

**Note** that each new LED position is indicated by a dash (-). The position is given in the x, y, and z coordinates of the object frame in meters.

If you would like to use your own marker positions file, place it in the `monocular_pose_tracker/marker_positions` folder and alter the launch file as explained below in the section 'Launch Files'.

#### Running the pose estimator with a USB camera 

To run the method with a USB camera it has to be calibrated. (ADD HOW TO CALIBRATE THEE CAMERA)

The information given (published) by the camera are the following two:

* mv_25001329/image_raw ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))

  The image from the camera. The LEDs will be detected in this image. 

* mv_25001329/camera_info ([sensor_msgs/CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html))

  The camera calibration parameters.

The pose estimator publishes the following topics:

* estimated_pose_UAV1 ([geometry_msgs/PoseWithCovarianceStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html))

     The estimated pose of the target object with respect to the camera. (in case of multiple UAVs: estimated_pose_UAV1, estimated_pose_UAV2, ...)

* image_with_detections ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))

  The image with the detected LEDs cirlced in green, the region of interest of the image that was processed bounded by a yellow rectangle, and the orientation trivectors (from all particles) projected onto the object.

* PoseParticles1 ([geometry_msgs/PoseArray])(http://docs.ros.org/api/geometry_msgs/html/msg/PoseArray.html)

	The pose of the (not resampled) particles with respect to the camera (in case of multiple UAVs: PoseParticles1, PoseParticles2, ...)

* ResampledParticles1  ([geometry_msgs/PoseArray])(http://docs.ros.org/api/geometry_msgs/html/msg/PoseArray.html)

	The pose of the particles with respect to the camera (in case of multiple UAVs: ResampledParticles1, ResampledParticles2, ...)

* FailFlag ([std_msgs/Float32MultiArray])(http://docs.ros.org/api/std_msgs/html/msg/Float32MultiArray.html)

	Fail flag (explained in monocular_pose_estimator.hpp)

* timePoseEst ([std_msgs/Duration])(http://docs.ros.org/api/std_msgs/html/msg/Duration.html)

	Time needed for processing the frame

* timeInitEst ([std_msgs/Duration])(http://docs.ros.org/api/std_msgs/html/msg/Duration.html)

	Time needed for the initialisation




#### Parameter Settings

The following parameters can be set dynamically during runtime. (Use 'rqt_reconfigure' to adjust the parameters).


* threshold_value (int, default: 240, min: 0, max: 255)

  This is the pixel intensity that will be used to threshold the image. All pixels with intensity values below this value will be set to zero. Pixels with intensity values equal to or higher than this value will retain their intensities.

* gaussian_sigma (double, default: 0.6, min: 0, max: 6)

  This is the standard deviation of the of the Gaussian that will be used to blur the image after being thresholded.

* min_blob_area (double, default: 20, min: 0, max: 100)

  This is the minimum blob area (in pixels squared) that will be detected as a cluster/LED.

* max_blob_area (double, default: 160, min: 0, max: 1000)

  This is the maximum blob area (in pixels squared) that will be detected as a cluster/LED. Clusters having an area larger than this will not be detected as LEDs.

* max_width_height_distortion (double, default: 0.7, min: 0, max: 1)

  This is a parameter related to the circular distortion of the detected clusters. It is the maximum allowable distortion of a bounding box around the detected cluster calculated as the ratio of the width to the height of the bounding rectangle. Ideally the ratio of the width to the height of the bounding rectangle should be 1. 

* max_circular_distortion (double, default: 0.7, min: 0, max: 1)

  This is a parameter related to the circular distortion of the detected clusters. It is the maximum allowable distortion of a bounding box around the detected cluster, calculated as the area of the cluster divided by pi times half the height or half the width of the bounding rectangle. 

* back_projection_pixel_tolerance (double, default: 5, min: 0, max: 100)

  This is the tolerance (in pixels) between a back projected LED and an image detection. If a back projected LED is within this threshold, then the pose for that back projection is deemed to be a possible candidate for the pose.

* nearest_neighbour_pixel_tolerance (double, default: 7, min: 0, max: 10)

  This is the tolerance for the prediction of the correspondences between object LEDs and image detections. If the predicted position of an LED in the image is within this tolerance of the image detections, then the LED and image detection are considered to correspond to each other.

* certainty_threshold (double, default: 1, min: 0, max: 1)

  This is the proportion of back projected LEDs into the image that have to be within the back_projection_pixel_tolerance, for a pose to be considered valid.

* valid_correspondence_threshold (double, default: 0.5, min: 0, max: 1)

  This is the ratio of all combinations of 3 of the correspondences that yielded valid poses (i.e., were within the certainty_threshold), for a set of correspondences to be considered valid.

* roi_border_thickness (int, default: 10, min: 10, max: 200)

  This is the thickness of the boarder around the region of interest in pixels

* number_of_occlusions (int, default: 0, min: 0, max: 5)

  This is the number of occlusion which can be introduced to check the Pose Estimator due to its robustness against occlusions. (The occlusions are randomly applied)

* number_of_false_detections (int, default: 0, min: 0, max: 30)
	
	This is the number of false detections which can be introduced to check the Pose Estimator due to its robustness against false detections (noise). (They are rnadomly applied inside the region of interest)

* active_markers (bool, default: true)

	This parameter defines if active or passive markers are used

* bUseParticleFilter (bool, default: true)

	This parameter defines if the particle filter approach is used, or if the IPE from Faessler et al. (ICRA 2014) is used

* N_Particle (int, default: 1, min: 1, max: 20000)

	This is the number of particles which is used in the particle filter. (Should not be done online!!!!)

* maxAngularNoise (int, default: 0, min: -1.5, max: 1.5)

	This is the maximum angular noise which is used in the motion model

* minAngularNoise (int, default: 0, min: -1.5, max: 1.5)

	This is the minimal angular noise which is used in the motion model

* maxTransitionNoise (int, default: 0, min: -0.5, max: 0.5)

	This is the maximum transition noise which is used in the motion model

* minTransitionNoise (int, default: 0, min: -0.5, max: 0.5)

	This is the minimal transition noise which is used in the motion model

* back_projection_pixel_tolerance_PF (int, default: 10, min: 0, max: 100)

	This is the tolerance (in pixels) between a back projected LED and an image detection during the weighting phase in the particle filter. If a back projected LED is within this threshold, the weight of the pose will be increased

* bMarkerNrX (bool, default: false)

	This parameter tells if the marker number X is downgraded or not. (Can only be used if one UAV is tracked)

* useOnlineExposeTimeControl (bool, default: false)

	If this parameter is ture, the exposure time will be adapted online. A change takes about 1ms, and is therefore not recommended.

* expose_time_base (int, default: 2000, min: 10, max: 8000

	This is the initial expose time which is used in case the expose time is calculated online

* bUseCamPos (bool, default: false)

	This boolean tells if the movement of the camera is considered in the pose prediction or not. (should only be used if reliable information is available)



#### Launch Files

The Pose Estimator needs to be launched with a launch file, since the location of the YAML configuration file containing the LED positions on the object needs to be specified.
An example launch file is presented below.


<launch> 

	<!-- Name of the YAML file containing the marker positions -->
	<arg name="YAML_file_name" default="nameOfTheMarkerPositionFile"/>

	<!-- File containing the the marker positions in the trackable's frame of reference -->
	<arg name="marker_positions_file" default="$(find monocular_pose_estimator)/marker_positions/$(arg YAML_file_name).yaml"/> 

	<group ns="monocular_pose_estimator" >
		<node name="monocular_pose_estimator" pkg="monocular_pose_estimator" type="monocular_pose_estimator" output="screen" respawn="false">  
			<rosparam command="load" file="$(arg marker_positions_file)"/>
			<param name= "threshold_value" value = "240" />
			<param name= "gaussian_sigma" value = "0.6" /> 
			<param name= "min_blob_area" value = "20" />
			<param name= "max_blob_area" value = "160" />
			<param name= "max_width_height_distortion" value = "0.7" />
			<param name= "max_circular_distortion" value = "0.7" />
			<param name= "back_projection_pixel_tolerance" value = "5" /> 
			<param name= "nearest_neighbour_pixel_tolerance" value = "7" /> 
			<param name= "certainty_threshold" value = "1" />
			<param name= "valid_correspondence_threshold" value = "0.5" /> 
			<param name= "number_of_occlusions" value = "0" />
			<param name= "number_of_false_detections" value = "0" />
			
			<!-- UAV SPECIFICATIONS -->													
			<param name= "active_markers" value = "True" />							
			<param name= "numUAV" value = "1" />										
			<param name= "numberOfMarkersUAV1" value = "5" />
			<param name= "numberOfMarkersUAV2" value = "0" />
			<param name= "numberOfMarkersUAV3" value = "0" />
			<param name= "numberOfMarkersUAV4" value = "0" />
			
			
			<!-- PARTICLE FILTER PARAMETER -->
			<param name= "bUseParticleFilter" value = "true" />
			<param name= "N_Particle" value = "100" />
			<param name= "maxAngularNoise" value = "0.015" /> 		
			<param name= "minAngularNoise" value = "-0.015" />		
			<param name= "maxTransitionNoise" value = "0.035" />		
			<param name= "minTransitionNoise" value = "-0.035" />	
			<param name= "back_projection_pixel_tolerance_PF" value = "4" /> 
			
			
			<!-- MARKER DOWNGRADE -->
			<param name= "bMarkerNr1" value = "False" />
			<param name= "bMarkerNr2" value = "False" />
			<param name= "bMarkerNr3" value = "False" />
			<param name= "bMarkerNr4" value = "False" />
			<param name= "bMarkerNr5" value = "False" />
			
			<!-- MISCELLANEOUS -->
			<param name= "useOnlineExposeTimeControl" value = "false" />			
			<param name= "expose_time_base" value = "2000" />		 
			<param name= "bUseCamPos" value = "false" />			
		</node> 
	
	 <node name="view_visualisation_image" pkg="image_view" type="image_view" args="image:=/monocular_pose_estimator/image_with_detections" />

	</group>

</launch>

In case the image and the camera info are coming from a bagfile, the file has to be added with the following lines
	<!-- rosbag play -->
	  <node pkg="rosbag" type="play" name="player" args=" path/of/the/bagfile/fileName.bag "/>

In case the some topics have to be recored the following lines can be added
	<!-- rosbag record -->
	 <node pkg="rosbag" type="record" name="record" args="-e 'topics/to/record' -o path/where/the/bagfile/will/be/saved/fileName.bag"/>
			


Important hints
---------------

#### Marker Positions
Every time the LED configuration is changed, the YAML file containing the marker positions has to be changed.
In case multiple UAVs are tracked, the makers have to be defined as only one UAV with more markers would be used
e.g.marker_positions:
  - x: 0.00	# start  of the first UAV
    y: 0.00
    z: 0.000
  - x: 0.123
    y: 0.00
    z: 0.00
  - x: 0.251
    y: 0.0
    z: 0.000
  - x: 0.12644
    y: 0.200454
    z: 0.00
  - x: 0.20644  
    y: 0.128304
    z: 0.13
  - x: 0.00	# start of the second UAV
    y: 0.00
    z: 0.000
  - x: 0.245
    y: 0.00
    z: 0.00
  - x: 0.245
    y: 0.25
    z: 0.000
  - x: 0.495
    y: 0.25
    z: 0.00
  - x: 0.245  
    y: 0.0
    z: 0.245

#### Exposure time
When new video-bagfiles are recorded or the pose estimator is used for an online application, the exposure time has to be set manually. Consider a too small exposure time may lead to tracking losses in bigger distances, while a too big one can lead to a very noisy environment and to tracking losses in closer distances.

#### Parameter changes
When using a provided launch file, some parameter change can have a big impact on the accuracy and the needed estimation time:
	- Number of particles
	- min/max noise in the motion model
	- min/max cluster size (detection stage)

#### Display images
Showing the 'image_with_detections' during calculations will lead to a not neglible increase of computational time (bias up to 0.01ms)


Run Experiments
--------------

To run simple experiments it is enough to run the launchfile provided above.
If a bag file has to be run, make sure the correct marker position file is used.
The bag files provided for testing the algorithm, require different marker position files, the corresponding launch file is already prepared for such a run. 







