# ROS2 node of multiscale marker detections

This ROS2 node detects multiscale marker consisting of Apriltag families.

For more information on AprilTag, the paper and the reference implementation: https://april.eecs.umich.edu/software/apriltag.html

[Acknowledgement] This code is forked from the Christian Rauch's ROS2 node implementation of Apriltag detection. [[link](https://github.com/christianrauch/apriltag_ros)]

## Installation 

First, create and move to the workspace directory where all projects related to ROS2 will be located. 
```
$ mkdir -p ros2_ws/src    # create ros2 workspace based on colcon build system
$ cd ros2_ws/src
```

Next, create a subdirectory of core codes for marker detection.
```
$ git clone --depth 1 --branch v3.2.0 https://github.com/AprilRobotics/apriltag.git
$ git clone https://github.com/tag-nav/multiscale-marker-detection.git
$ cp -v multiscale-marker-detection/tagCustom52h12.* apriltag
$ rm -rf multiscale-marker-detection
```

Then, clone codes needed for ROS2 node generation.
```
$ git clone https://github.com/tag-nav/multiscale-marker-detection-ros2.git
$ git clone https://github.com/christianrauch/apriltag_msgs
```

Now, build all the projects under the workspace folder.
```
$ cd ..
$ colcon build
```

## Execution

First, source the ROS2 workspace.
```
$ source install/setup.bash
```

Next, move to where the ROS2 package for marker detection is located.
```
$ cd src/multiscale-marker-detection-ros2/
```

Then, for detecting non-nested layouts (based on `Classic36h11`), run the following command with proper names of image and camera information topics.
```
ros2 run apriltag_ros apriltag_node --ros-args \
    -r image_rect:=/camera/image \
    -r camera_info:=/camera/camera_info \
    --params-file `ros2 pkg prefix apriltag_ros`/cfg/Classic36h11.yaml
```

Or, for detecting nested layouts (based on `Custom52h12`), run the following command with proper names of image and camera information topics.
```
ros2 run apriltag_ros apriltag_node --ros-args \
    -r image_rect:=/camera/image \
    -r camera_info:=/camera/camera_info \
    --params-file `ros2 pkg prefix apriltag_ros`/cfg/Custom52h12.yaml
```

## Topics

### Subscriptions:
The node subscribes via a `image_transport::CameraSubscriber` to rectified images on topic `image_rect`. The set of topic names depends on the type of image transport (parameter `image_transport`) selected (`raw` or `compressed`):
- `image_rect` (`raw`, type: `sensor_msgs/msg/Image`)
- `image_rect/compressed` (`compressed`, type: `sensor_msgs/msg/CompressedImage`)
- `camera_info` (type: `sensor_msgs/msg/CameraInfo`)

### Publisher:
- `/tf` (type: `tf2_msgs/msg/TFMessage`)
- `detections` (type: `apriltag_msgs/msg/AprilTagDetectionArray`)

The camera intrinsics `P` in `CameraInfo` are used to compute the marker tag pose `T` from the homography `H`. The image and the camera intrinsics need to have the same timestamp.

The tag poses are published on the standard TF topic `/tf` with the header set to the image header and `child_frame_id` set to either `tag<family>:<id>` (e.g. "tag36h11:0") or the frame name selected via configuration file. Additional information about detected tags is published as `AprilTagDetectionArray` message, which contains the original homography  matrix, the `hamming` distance and the `decision_margin` of the detection.

## Configuration

The node is configured via a yaml configurations file. For the complete ROS yaml parameter file syntax, see: https://github.com/ros2/rcl/tree/master/rcl_yaml_param_parser.

The configuration file has the format:
```yaml
apriltag:                 # node name
  ros__parameters:
    # setup (defaults)
    image_transport: raw  # image format: "raw" or "compressed"
    family: 36h11         # tag family name: 16h5, 25h9, 36h11
    size: 1.0             # default tag edge size in meter
    profile: false        # print profiling information to stdout

    # tuning of detection (defaults)
    max_hamming: 0        # maximum allowed hamming distance (corrected bits)
    detector:
      threads: 1          # number of threads
      decimate: 2.0       # decimate resolution for quad detection
      blur: 0.0           # sigma of Gaussian blur for quad detection
      refine: 1           # snap to strong gradients
      sharpening: 0.25    # sharpening of decoded images
      debug: 0            # write additional debugging images to current working directory

    # (optional) list of tags
    # If defined, 'frames' and 'sizes' must have the same length as 'ids'.
    tag:
      ids:    [<id1>, <id2>, ...]         # tag IDs for which to publish transform
      frames: [<frame1>, <frame2>, ...]   # frame names
      sizes:  [<size1>, <size1>, ...]     # tag-specific edge size, overrides the default 'size'
```

The `family` (string) defines the tag family for the detector and must be one of `16h5`, `25h9`, `36h11`, `Circle21h7`, `Circle49h12`, `Custom48h12`, `Standard41h12`, `Standard52h13`. `size` (float) is the tag edge size in meters, assuming square markers.

Instead of publishing all tag poses, the list `tag.ids` can be used to only publish selected tag IDs. Each tag can have an associated child frame name in `tag.frames` and a tag specific size in `tag.sizes`. These lists must either have the same length as `tag.ids` or may be empty. In this case, a default frame name of the form `tag<family>:<id>` and the default tag edge size `size` will be used.

The remaining parameters are set to the their default values from the library. See `apriltag.h` for a more detailed description of their function.

See [tags_36h11.yaml](cfg/tags_36h11.yaml) for an example configuration that publishes specific tag poses of the 36h11 family.
