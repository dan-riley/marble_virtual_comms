#include <MarbleVirtualComms.h>

namespace marble_virtual_comms {

MVCPeerData::MVCPeerData() {
  seq = 0;
  last_packet = 0;
  packet_total = 0;
  recvd_packets = 0;
  data_length = 0;
}

MVCPeerData::~MVCPeerData() {
}

void MVCPeerData::init(uint16_t seq_, uint16_t packet_total_) {
  time = ros::Time::now();
  seq = seq_;
  packet_total = packet_total_;
  packets.resize(packet_total);
}

void MVCPeerData::addPacket(std::string data, uint16_t packet_num) {
  packets[packet_num] = data;
  recvd_packets++;
  data_length += data.length() - HEADER_SIZE;
}

MVCPeerTopic::MVCPeerTopic() {
}

MVCPeerTopic::~MVCPeerTopic() {
}

MVCPeer::MVCPeer() {
}

MVCPeer::~MVCPeer() {
}

void MVCPeer::init(std::string name_, MarbleVirtualComms& parent_) {
  parent = &parent_;
  name = name_;
  initTopics();

  ros::NodeHandle pnh("~");

  if (name == "all") {
    // "All" only subscribes to broadcast topics, and doesn't publish locally
    topics[0].sub = pnh.subscribe<marble_multi_agent::AgentMsg>(parent->sendpre + "ma_data", 100, &MarbleVirtualComms::maDataCallback, parent);
  } else {
    // Add publishers and subscribers for each topic
    std::string sendpre = parent->sendpre + name;
    std::string recvpre = parent->recvpre + name;

    // Create the ma_data publisher
    topics[0].pub = pnh.advertise<marble_multi_agent::AgentMsg>(recvpre + "/ma_data", 100);

   // Create pair for DM Requests
    topics[1].sub = pnh.subscribe<marble_multi_agent::DMReqArray>(sendpre + "/dm_request", 100, boost::bind(&MarbleVirtualComms::dmReqCallback, parent, _1, name));
    topics[1].pub = pnh.advertise<marble_multi_agent::DMReqArray>(recvpre + "/dm_request", 100);

    // Create pair for DM Responses
    topics[2].sub = pnh.subscribe<marble_multi_agent::DMRespArray>(sendpre + "/dm_response", 100, boost::bind(&MarbleVirtualComms::dmRespCallback, parent, _1, name));
    topics[2].pub = pnh.advertise<marble_multi_agent::DMRespArray>(recvpre + "/dm_response", 100);
  }
}

void MVCPeer::initTopics() {
  topics.resize(3);
  for (auto i = 0; i < 3; ++i) {
    topics[i].msg_type = i;
    topics[i].seq = 0;
  }
}
}
