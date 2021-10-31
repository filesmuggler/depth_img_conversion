# ip_conversion

## Usage

### Get PNG from rosbag

Convert data from rgbd camera to png set

```
roslaunch ip_rosbag_to_img convert_ros2img.launch path_to_bag:=<path_to_bag_file>
```

### Publish PNG alongside rosbag

Publish topic composed of PNG images for verification or pointcloud generation.
```
roslaunch ip_rosbag_to_img test_img2ros.launch path_to_bag:=<path_to_bag_file>
```

## Results

Gif presenting reconstruction of the depth image from PNG file (bottom) to ROS topic. A little delay is observed due to conversion time.

![Alt Text](./media/depth.gif)

Video below presents reconstruction of pointcloud data from republished depth image topic. White is original and pink is reconstructed one.

[![Watch the video](https://i.imgur.com/vKb2F1B.png)](./media/pcl.mkv)


## Bottomline

It's not finished yet :)))
Lots of work to be done 
<br />
<img src="https://media.giphy.com/media/E6jscXfv3AkWQ/giphy.gif" width="250" height="250"/> </img>
<a href="https://giphy.com/gifs/cat-typing-E6jscXfv3AkWQ">via GIPHY</a></p>