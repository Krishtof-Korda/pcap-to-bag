#include <iostream>
#include <ouster_ros/PacketMsg.h>
#include <rosbag/bag.h>
#include <tins/ip_reassembler.h>
#include <tins/packet.h>
#include <tins/rawpdu.h>
#include <tins/sniffer.h>
#include <tins/tins.h>
#include <tins/udp.h>
#include <vector>

// Input the imu packet payload size based on your pcap imu udp packet size.
// e.g. 48 for standard imu packet.
uint32_t imu_payload_size = 48;

void read_packets(const std::string &pcap_filename,
                  const std::string &bag_filename,
                  const std::string &num_channels) {

  uint32_t num_chans = static_cast<uint32_t>(
      std::stoul(num_channels)); // Number of channels, e.g. for an OSX-64,
                                 // num_channels = 64

  uint32_t azimuths_per_packet = 16;
  uint32_t num_header_footer_words = 5;
  uint32_t bytes_per_word = 4;
  uint32_t words_per_channel = 3;
  uint32_t lidar_payload_size =
      azimuths_per_packet * bytes_per_word *
      (num_header_footer_words + (num_chans * words_per_channel));

  rosbag::Bag bag;
  bag.open(bag_filename, rosbag::bagmode::Write);

  Tins::IPv4Reassembler reassembler;
  Tins::FileSniffer sniffer(pcap_filename);

  std::vector<uint8_t> buf;

  size_t lidar_packets = 0;
  size_t imu_packets = 0;

  std::chrono::nanoseconds tic(0), toc(0);

  while (Tins::Packet packet = sniffer.next_packet()) {
    auto &pdu = *packet.pdu();
    const Tins::Timestamp &timestamp = packet.timestamp();
    if (reassembler.process(pdu) != Tins::IPv4Reassembler::FRAGMENTED) {
      const Tins::UDP *udp = pdu.find_pdu<Tins::UDP>();
      if (!udp) {
        continue;
      }
      const Tins::RawPDU *raw = pdu.find_pdu<Tins::RawPDU>();
      if (!raw) {
        continue;
      }
      const Tins::RawPDU::payload_type &payload = raw->payload();

      std::chrono::nanoseconds ns((std::chrono::microseconds)timestamp);
      if (tic.count() == 0)
        tic = ns;
      if (ns < tic)
        tic = ns;
      if (ns > toc)
        toc = ns;
      ros::Time ros_time;
      ros_time.fromNSec(ns.count());

      ouster_ros::PacketMsg msg;
      msg.buf = payload;
      msg.buf.push_back(0);

      if (payload.size() == lidar_payload_size) {
        bag.write("/os_node/lidar_packets", ros_time, msg);
        lidar_packets++;
      } else if (payload.size() == imu_payload_size) {
        bag.write("/os_node/imu_packets", ros_time, msg);
        imu_packets++;
      }
    }
  }
  std::cout << "Written " << lidar_packets << " lidar packets and "
            << imu_packets << " imu packets" << std::endl;
  std::cout << "Duration: "
            << std::chrono::duration_cast<std::chrono::duration<float>>(toc -
                                                                        tic)
                   .count()
            << " seconds" << std::endl;
}

int main(int argc, char** argv) {
    if (argc != 4) {
        std::cout << "USAGE: pcap_to_bag pcap_filename.pcap bag_filename.bag num_lidar_channels"
                  << std::endl;
        return 1;
    }
    read_packets(argv[1], argv[2], argv[3]);
    return 0;
}
