# Ouster pcap to rosbag tool
This tool converts Ouster pcaps to rosbags containg only the packets in the `/os_node/lidar_packets` and `/os_node/imu_packets` topics.

If you would like `PointCloud2` msgs you will need to run the resulting bagfile through the `ouster_ros` node and record the `/os_cloud_node/points` topic.

## Requirements

* `libpcap-dev`
* `libtins-dev`

Install requirements prior to building using the command below.
```
 sudo apt install libpcap-dev libtins-dev
 ```

## Building

You will create a new catkin workspace and link both the `ouster_example` and `pcap-to-bag` code repos.
The code repos can live anywhere on your machine, just make sure to use absolute paths like **`/home/user/pcap-to-bag`** and **do not** use relative paths such as `~/pcap-to-bag`
> **_NOTE:_** You must use ouster_example at [tag 20210608](https://github.com/ouster-lidar/ouster_example/releases/tag/20210608)

* `mkdir -p myworkspace/src`
*  `cd myworkspace`
*  `ln -s /path/to/ouster_example ./src/`
*  `ln -s /path/to/pcap-to-bag ./src/`
*  `catkin_make -DCMAKE_BUILD_TYPE=Release`

## Running

* After building, the `pcap_to_bag` executable will be in this folder: `myworkspace/devel/lib/pcap_to_bag/`
* You can run `./pcap_to_bag name_of_pcap.pcap name_of_bag.bag number_of_laser_channels`
* e.g. for a 128 channel sensor `./pcap_to_bag my.pcap my.bag 128`
