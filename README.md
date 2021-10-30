# ip_conversion

## Usage

### Get PNG from rosbag

Convert data from rgbd camera to png set

```
roslaunch ip_rosbag_to_img convert_ros2img.launch path_to_bag:=<path_to_bag_file>
```

### Publish PNG alongside rosbag

Publish topic composed of PNG images for verification or pointcloud generation (not implemented yet)
```
roslaunch ip_rosbag_to_img test_img2ros.launch path_to_bag:=<path_to_bag_file>
```

## Results

Gif presenting reconstruction of the depth image from PNG file (bottom) to ROS topic. A little delay is observed due to conversion time.

![Alt Text](./media/depth.gif)

## Bottomline

It's not finished yet :)))
Lots of work to be done 
<iframe src="https://giphy.com/embed/E6jscXfv3AkWQ" width="600" height="250" frameBorder="0" class="giphy-embed" allowFullScreen></iframe><p><a href="https://giphy.com/gifs/cat-typing-E6jscXfv3AkWQ">via GIPHY</a></p>

