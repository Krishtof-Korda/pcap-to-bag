#pcap to rosbag tool

This tool converts pcaps to rosbags.

## Requirements

* `libpcap-dev`
* `libtins-dev`

## Building

* `mkdir -p myworkspace/src && cd myworkspace && ln -s /path/to/ouster_example ./src/ && ln -s /path/to/pcap-to-bag ./src/ && catkin_make -DCMAKE_BUILD_TYPE=Release`

## Running

* After building, the `pcap_to_bag` executable will be in this folder: `myworkspace/devel/lib/pcap_to_bag/`
* You can run `./pcap_to_bag name_of_pcap.pcap name_of_bag.bag number_of_laser_channels`
* e.g. for a 128 channel sensor `./pcap_to_bag my.pcap my.bag 128`
