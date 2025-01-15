# bea_visioscan_rd1
ROS1 driver for VISIOSCAN RD

# ROS Driver for VISIOSCAN

The document describes the usage of the driver for BEA VISIOSCAN laser scanner.

The driver is based upon the [Boost Asio library](http://www.boost.org)

## Supported LIDAR

| Lidar Model            |
| ---------------------- |
| Visioscan RD           |

Visit following website for more details about Visioscan laser scanner: <https://asia.beasensors.com/en/product/lzr-visioscan-rd/>

## Tested environment

The driver is only tested within a VirtualBox VM installing environment below:

OS: [Ubuntu 20.04.6 LTS (Focal Fossa)](https://www.releases.ubuntu.com/20.04/)

ROS: [Noetic Ninjemys](https://wiki.ros.org/noetic/)

## Usage with ROS

### Create a ROS workspace

1. For example, choose the directory name `catkin_ws`, for "development workspace" :

   ```bash
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws/src
   ```

2. Copy & Paste project files into `/catkin_ws/src/` directory
   

### Parameters

The parameters for configuring laser scanner are listed as below. You need to change the parameters in the file `/launch/visioscan_rd.launch` or in the `bea_node.cpp` file corresponding to the configuration in the laser scanner.

| Parameter       | Description |
| --------------- | ----------- |
| frame_id        | Frame ID for LaserScan msg                                 |
| topic_id        | Topic ID for the publisher                                 |
| laser_ip        | IP address of laser scanner                                |
| laser_port      | Ethernet port number of laser scannner                     |
| laser_direction | Indicate if laser scanner is normal mounted or upside down |

### Compile & install visioscan_rd package

1. Build visioscan_rd package
   From the root of your workspace `catkin_ws`, you can now build visioscan_rd package using the command:

   ```bash
   cd ~/catkin_ws/
   catkin_make
   ```

2. Package environment setup
    
    ```bash
    source ./devel/setup.bash
    ```

    Note: Add permanent workspace environment variables.
    It's convenient if the ROS environment variables are automatically added to your bash session every time a new terminal is launched:

    ```bash
    echo "source catkin_ws/devel/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```

### Run visioscan_rd

You can run `visioscan_rd` node with following steps, or make a [Quick start](#quick-start)

#### Step 1. Run roscore

    ```bash
    roscore
    ```

#### Step 2. Run bea_node

Open another terminal and run following command:

   ```bash
   rosrun visioscan_rd bea_node
   ```

#### Step 3: Run RViz

You can run `RViz` package to visualize the point cloud. Open a new terminal and run the following command:

   ```bash
   rosrun rviz rviz
   ```
   Note: Please make sure that the frame ID and scan topic ID are corresponding to that you set in the code or the config file.


### Quick start

After compiled and installed the package, run the following command to make a quick start:

    ```bash
    roslaunch visioscan_rd visioscan_rd.launch
    ```

