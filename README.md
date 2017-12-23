- [Self Driving Car System Integration](#sec-1)
- [Team: Total Recall Dryvers](#sec-2)
- [Structure](#sec-3)
- [Hardware Requirement](#sec-4)
- [Software Requirement](#sec-5)
- [Build](#sec-6)
  - [Without Docker](#sec-6-1)
  - [With Docker](#sec-6-2)
    - [Running](#sec-6-2-1)
    - [Cleaning](#sec-6-2-2)
- [Real world testing](#sec-7)

# Self Driving Car System Integration<a id="sec-1"></a>

This is the final capstone project in Udacity's Self-Driving Car Nanodegree. In this project, we write code that will automatically drive a "Carla." The Carla is an actual self-driving car equipped with necessary sensors and the drive-by-wire module.

# Team: Total Recall Dryvers<a id="sec-2"></a>

-   [Luca Venturi](https://github.com/lucav76)
-   [Jochen](https://github.com/jocmom)
-   [Tim Aske](https://github.com/TimSoft77)
-   [Krishtof Korda](https://github.com/Krishtof-Korda)
-   [Kyung Mo Kweon](https://github.com/kkweon)

# Structure<a id="sec-3"></a>

Our project consists of 3 modules: **perception**, **planning**, and **control**.

<img src="./readme_assets/structure.png" alt="structure" />

# Hardware Requirement<a id="sec-4"></a>

The Carla has the following hardware spec

-   31.4 GiB Memory
-   Intel Core i7-6700K CPU @ 4 GHz x 8
-   TITAN X Graphics
-   64-bit OS

# Software Requirement<a id="sec-5"></a>

Another requirement is that the project cannot include external libraries that are not included in this repository. To run this project, you will need

-   Ubuntu 14.04+
-   ROS
    -   [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
    -   [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
-   [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
    -   Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install binary](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
-   Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases/tag/v1.2).

# Build<a id="sec-6"></a>

## Without Docker<a id="sec-6-1"></a>

1.  Clone the project repository

    ```bash
    git clone https://github.com/udacity/CarND-Capstone.git
    ```
2.  Install python dependencies

    ```bash
    cd CarND-Capstone
    pip install -r requirements.txt
    ```
3.  Make and run styx

    ```bash
    cd ros
    catkin_make
    source devel/setup.sh
    roslaunch launch/styx.launch
    ```
4.  Run the simulator

    ```bash
    ./run_udacity_simulator
    ```

## With Docker<a id="sec-6-2"></a>

### Running<a id="sec-6-2-1"></a>

```bash
docker-compose up
```

### Cleaning<a id="sec-6-2-2"></a>

```bash
docker-compose down -v
# or if you want to clean up entirely including the build image
docker-compose down -v --rmi all
```

# Real world testing<a id="sec-7"></a>

1.  Download [training bag](https://drive.google.com/file/d/0B2_h37bMVw3iYkdJTlRSUlJIamM/view?usp=sharing) that was recorded on the Udacity self-driving car (a bag demonstraing the correct predictions in autonomous mode can be found [here](https://drive.google.com/open?id=0B2_h37bMVw3iT0ZEdlF4N01QbHc)
2.  Unzip the file

    ```bash
    unzip traffic_light_bag_files.zip
    ```
3.  Play the bag file

    ```bash
    rosbag play -l traffic_light_bag_files/loop_with_traffic_light.bag
    ```
4.  Launch your project in site mode

    ```bash
    cd CarND-Capstone/ros
    roslaunch launch/site.launch
    ```
5.  Confirm that traffic light detection works on real life images
