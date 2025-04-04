# Inspire Hand ROS Driver

- ROS driver for dexterous inspire hand
- 

## TODO

- [x] Support two hand cases
- [ ] Publish Vision Pro Results as TF
- [ ] Publish force sensor values


## Environment Setup

```
conda create -n ros python=3.10
conda activate ros
conda install -c conda-forge rospkg catkin_pkg cmake
pip install rosinstall_generator wstool rosinstall six vcstools msgpack empy==3.3.4
conda install pinocchio -c conda-forge
pip install avp_stream tqdm tyro lark
pip install dex_retargeting

cd /home/seung/ros2_ws && colcon build --symlink-install --cmake-args -DPYTHON_EXECUTABLE=$CONDA_PREFIX/bin/python
```

## Run

```
# Ensure you have permissions to access the USB device
sudo chmod 777 /dev/ttyUSB0
sudo chmod 777 /dev/ttyUSB1
```

### By launch file
```
# left hand
ros2 launch inspire_hand_driver left_hand.launch.py
# right hand
ros2 launch inspire_hand_driver right_hand.launch.py
# both hands
ros2 launch inspire_hand_driver both_hand.launch.py
```


# test open close
ros2 run inspire_hand_driver test_open_close hand_type:='right'
ros2 run inspire_hand_driver test_retargeting hand_type:='left'


# run Vision Pro's hand tracker
ros2 run inspire_hand_driver cmd_from_vision_pro hand_type:='both'

```