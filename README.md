# Spatio-Temporal Voxel Layer [![Build Status](http://build.ros.org/buildStatus/icon?job=Kdev__spatio_temporal_voxel_layer__ubuntu_xenial_amd64)](http://build.ros.org/view/Kbin_uX64/job/Kdev__spatio_temporal_voxel_layer__ubuntu_xenial_amd64/)

This is a drop in replacement for the voxel_grid voxel representation of the environment. This package does a number of things to improve on the voxel grid package and extend the capabilities offered to the users, under a LGPL v2.1 license. Developed and maintained by [Steven Macenski](https://www.linkedin.com/in/steven-macenski-41a985101/) at [Simbe Robotics](http://www.simberobotics.com/).

This package sits on top of [OpenVDB](http://www.openvdb.org/), an open-source C++ library built by Dreamworks "comprising a novel hierarchical data structure and a suite of tools for the efficient storage and manipulation of sparse volumetric data discretized on three-dimensional grids. It is developed and maintained by DreamWorks Animation for use in volumetric applications typically encountered in feature film production."

Leveraging OpenVDB, we have the ability to efficiently maintain a 3 dimensional voxel-representative world space. We wrap this with ROS tools and interfaces to the [navigation stack](http://wiki.ros.org/navigation) to allow for use of this layer in standard ROS configurations. It is certainly possible to utilize this package without ROS/Navigation and I invite other competing methodologies to develop here and create interfaces. 

An simple example video can be seen [here](https://www.youtube.com/watch?v=8YIFPiI1vrg&feature=youtu.be), another more complex example can be seen [here](https://youtu.be/hpWxa6M1AR8) with global decay and frustum decay while moving with dynamic objects. A short clip from this is shown below.

![ezgif com-video-to-gif](https://user-images.githubusercontent.com/14944147/37010885-b18fe1f8-20bb-11e8-8c28-5b31e65f2844.gif)

We found in experimental trials with **6** 7hz dense stereo RGBD cameras we ran the major move_base process at **20-50%** nominal from **80-110%** on a 5th gen i7 CPU in the global costmap updating using the existing `voxel_layer`.  

## **Spatio**-
The Spatio in this package is the representation of the environment in a configurable `voxel_size` voxel grid stored and searched by OpenVDB. 

In addition, buffered measurement readings have the option to run an approximate voxel grid filter, parameterizable at runtime in the configuration yamls. It is incredibly useful to reduce spikes in `move_base` cpu due to dense measurement readings when getting close to objects (i.e. more points), but does increase the overhead very slightly (1-2%) for nominal operations. It's a trade off but I recommend using it. 

Below is an example a size of map that is **trivial** for the Spatio-Temportal Voxel Grid to maintain and render. This accounts for a 60,000 sq.ft. retail store with 710,765 voxels at a 0.05m resolution, with a size in memory of a mere 6.45MB.  

![full_sore](https://user-images.githubusercontent.com/14944147/37013097-11e4f782-20c6-11e8-8212-6fca6e54331c.png)

## -**Temporal**
The Temporal in this package is the novel concept of `voxel_decay` whereas we have configurable functions that expire voxels and their occupation over time. Infrasture was created to store times in each voxel after which the voxel will disappear from the map. This is combined with checking inclusion of voxels in current measurement frustums to accelerate the decay of those voxels that do not have measurements but should if still in the scene and remain marked. This is done rather than simply clearing them naively or via costly raytracing. The time it takes to clear depends on the configured functions and acceleration factors.

Voxel acceleration uses given FOV to compute traditional 6-planed cubical frustums. Sensors that cannot be modelled by traditional frustums (i.e. 360 lidars or sensors without flat back planes) the frustum acceleration mathematics breakdown, **do not use frustum acceleration for these class of sensors**. PRs to implement curved frustums are welcome and encouraged.  

Future extensions will also to query a static map and determine which connected components belong to the map, not in the map, or moving. Each of these three classes of blobs will have configurable models to control the time they persist, and if these things are reported to the user.

Below is an example of instantaneous decay, where readings in frustum are accelerated and decayed at each iteration. The models provided can be tuned to do this, or persist through linear or exponental equations. The second example has the acclerated frustum with tuned decay times and acceleration factors in navigation mode.

![ezgif com-video-to-gif 1](https://user-images.githubusercontent.com/14944147/37063574-d0923d24-2167-11e8-850c-18b6aed61634.gif)

![ezgif com-video-to-gif 3](https://user-images.githubusercontent.com/14944147/37127014-cacc1d1c-2241-11e8-8c2e-6ff7341333c9.gif)

## Local Costmap
This package utilizes all of the information coming in from the robot before the decay time for the local costmap. Rather than having a defined, discrete spatial barrier for the local planner to operate in, it instead relies on the user configuration of the layer to have a short decay time of voxels (1-30 seconds) so that you only plan in relavent space. This was a conscious design requirement since frequently the local planner should operate with more information than other times when the speed is greater or slower. This natively implements dynamic costmap scaling for speed.

It is the user's responsibility to chose a decay time that makes sense for your robot's local planner. 5-15 seconds I have found to be nominally good for most open-sourced local planner plugins. I do not recommend using this for planar lidars, 2D raytracing for professional grade lidars is sufficiently efficient and effective.

## Global Costmap
Similar to the local costmap, the amount of information you want to store due to entropy in your scenes depend on your use-case. It is certainly possible to **not** decay the voxels in the global map at all. However, in practical application, I find a time 15-45 seconds to be a good balance due to things moving in the scene (i.e. store, warehouse, construction zone, office, etc). Permanent voxels set decay to -1. I do not recommend using this for planar lidars, 2D raytracing for professional grade lidars is sufficiently efficient and effective.

## Mapping

As the images above suggest, you can use this to map an environment in 3D in realtime if you choose. If you enable mapping mode, then it will maintain the entire voxel grid and you can save the map using the services provided. At the moment, I support mapping but there is no probabilistic (yet!) marking framework, so what the sensor sees is what the map gets. This is likely to change in the near to middle term future as 3D localization becomes more interesting to the enterprise robotics community. 

You can run multiple instances of the layer one to map and other to navigate if you want to navigate while mapping the environment. Mapping will also save incremental maps in the launch directory. Maps may be visualized using `vdb_viewer`. The costmap and occupancy point clouds will not generate in this mode from this layer. Utility functions are provided so you don't need to learn anything about vdb files to convert to a pcl pointcloud in `vdb2pc.hpp`. 

If you would like to be involved in this work, I would gladly take contributors and coauthors.

## Installation
Required dependencies ROS Kinetic, navigation, OpenVDB, TBB.

If you are familiar with ROS, I have setup rosdep to work with all the necessary libraries, no need to install manually. In your workspace:

`sudo rosdep init && rosdep update && rosdep install --from-paths src --ignore-src -r -y`

If you are not familiar or cannot get rosdep to work, the dependencies and docs are listed below. 

### ROS

[See install instructions here.](http://wiki.ros.org/kinetic/Installation)

### Navigation

`sudo apt-get install ros-kinetic-navigation`

### OpenVDB

`sudo apt-get install libopenvdb3.1 libopenvdb-dev libopenvdb-tools`

### TBB

`sudo apt-get install libtbb-dev libtbb2`

### OpenEXR

`sudo apt-get install libopenexr-dev`

## Configuration and Running

### costmap_common_params.yaml

An example fully-described configuration is shown below.

```
rgbd_obstacle_layer:
  enabled:               true
  voxel_decay:           20     #seconds if linear, e^n if exponential
  decay_model:           0      #0=linear, 1=exponential, -1=persistent
  voxel_size:            0.05   #meters
  track_unknown_space:   true   #default space is unknown
  observation_persistence: 0.0  #seconds
  max_obstacle_height:   2.0    #meters
  unknown_threshold:     15     #voxel height
  mark_threshold:        0      #voxel height
  update_footprint_enabled: true
  combination_method:    1      #1=max, 0=override
  obstacle_range:        3.0    #meters
  origin_z:              0.0    #meters
  publish_voxel_map:     true   # default off
  transform_tolerance:   0.2    # seconds
  mapping_mode:          false  # default off, saves map not for navigation
  map_save_duration:     60     #default 60s, how often to autosave
  observation_sources:   rgbd1_clear rgbd1_mark
  rgbd1_mark:
    data_type: PointCloud2
    topic: camera1/depth/points
    marking: true
    clearing: false
    min_obstacle_height: 0.3     #default 0, meters
    max_obstacle_height: 2.0     #defaule 3, meters
    expected_update_rate: 0.0    #default 0, if not updating at this rate at least, remove from buffer
    observation_persistence: 0.0 #default 0, use all measurements taken during now-value, 0=latest 
    inf_is_valid: false          #default false, for laser scans
    clear_after_reading: true    #default false, clear the buffer after the layer gets readings from it
    voxel_filter: true           #default off, apply voxel filter to sensor, recommend on 
  rgbd1_clear:
    data_type: PointCloud2
    topic: camera1/depth/points
    marking: false
    clearing: true
    min_z: 0.1                   #default 0, meters
    max_z: 7.0                   #default 10, meters
    vertical_fov_angle: 0.7      #default 0.7, radians
    horizontal_fov_angle: 1.04   #default 1.04, radians
    decay_acceleration: 1.       #default 0, 1/s^2. If laser scanner MUST be 0
```

### local/global_costmap_params.yaml

Add this plugin to your costmap params file. 

`- {name: rgbd_obstacle_layer,     type: "spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer"}`


### Running

`roslaunch [navigation_pkg] move_base.launch`


### Debug and Model Fitting

I have made the frustum transformations available for visualization and debugging. You may enable them by the `VISUALIZE_FRUSTUM` macro, though be aware it takes a substantial decrease on performance since we're creating and destroying a ros publisher at a non-trivial rate.

This can also be used for situations where you do not know your camera's proper frustum FOVs. It is possible to enable it and tweek the FOVs until you get the appropriate coverage of the space your sensor carves out in the global space. You should only do this with one sensor at a time or else your frustum in rviz might jitter around. ;-)

### Interesting side note

We are able to iterate over very large grids for voxel decay, however there is clearly for every frequency (running at 1, 5, 10, 100hz) an upper limit. In the image below, we don't actually hit the limit of the data structure, but iterating at 2hz, we hit the limit of ROS' ability to publish a sufficiently large point cloud in that  time period, we are still running but you can see the robot at the end of an aisle without occupancy points, but still costmap marking from the underlying grid.

To counter this I include a service to save the grid in the .vdb format for later visualization, and for this reason I do not recommend visualizing the grid during nominal operations unless your decay time is relatively low (0-15 seconds) or else the layer may not meet its frequency requirements due to publishing this massive pointcloud. 

![openvdb2](https://user-images.githubusercontent.com/14944147/37010656-8ce4ff4c-20ba-11e8-9c35-1ce3e3039f77.png)

