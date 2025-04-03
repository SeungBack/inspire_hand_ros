# Inspire Hand ROS Driver

- ROS driver for dexterous inspire hand
- 

## TODO

- [ ] Publish Vision Pro Results as TF
- [ ] Support two hand cases
- [ ] Add action server
- [ ] Support force values


## Environment Setup

```
conda create -n ros python=3.10
conda activate ros
conda install -c conda-forge rospkg catkin_pkg cmake
pip install rosinstall_generator wstool rosinstall six vcstools msgpack empy==3.3.4
conda install pinocchio -c conda-forge
pip install avp_stream tqdm tyro
pip install dex_retargeting

cd /home/seung/ros2_ws && colcon build --symlink-install --cmake-args -DPYTHON_EXECUTABLE=$CONDA_PREFIX/bin/python
```

## Run

```
# Ensure you have permissions to access the USB device
sudo chmod 777 /dev/ttyUSB0

# run the driver
ros2 launch inspire_hand_description view_hand.launch.py
ros2 run inspire_hand_driver inspire_hand_driver

# test open close
ros2 run inspire_hand_driver test_open_close 

# run Vision Pro's hand tracker
ros2 run inspire_hand_driver cmd_from_vision_pro 
```