#ifndef MVCPEER_MVCPEER_H
#define MVCPEER_MVCPEER_H

#include <ros/ros.h>

namespace marble_virtual_comms {

class MarbleVirtualComms;


class MVCPeerData {

public:
  MVCPeerData();
  virtual ~MVCPeerData();
  void init(uint16_t seq, uint16_t packet_total);
  void addPacket(std::string data, uint16_t packet_num);

  // Buffer for message received
  std::vector<std::string> packets;
  // Tracking variables for received data
  ros::Time time;
  uint16_t seq;
  uint16_t last_packet;
  uint16_t packet_total;
  uint16_t recvd_packets;
  uint32_t data_length;
};


class MVCPeerTopic {

public:
  MVCPeerTopic();
  virtual ~MVCPeerTopic();

  uint8_t msg_type;
  // Publisher to write received data to mesh_comm/recv locally
  ros::Publisher pub;
  // Subscriber to read data written to mesh_comm/send to send to this agent
  ros::Subscriber sub;
  // Buffers for messages
  uint16_t seq;
  std::map<uint16_t, MVCPeerData> sbuffer;
  std::map<uint16_t, MVCPeerData> rbuffer;
};


class MVCPeer {

public:
  MVCPeer();
  virtual ~MVCPeer();
  void init(std::string name_, MarbleVirtualComms& parent);

  // Name of this agent
  std::string name;
  // Topics for this peer
  std::vector<MVCPeerTopic> topics;

protected:
  void initTopics();

  MarbleVirtualComms* parent;
};
}
#endif
