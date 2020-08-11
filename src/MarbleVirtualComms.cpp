#include <MarbleVirtualComms.h>

namespace marble_virtual_comms {

MarbleVirtualComms::MarbleVirtualComms(const ros::NodeHandle private_nh_, const ros::NodeHandle &nh_) : nh(nh_), private_nh(private_nh_) {

  private_nh.param("vehicle", id, std::string("Base"));
  private_nh.param("timeout", timeout, 3.0);
  private_nh.param("timer", timer, 0.2);

  client = std::make_unique<subt::CommsClient>(id, false);

  if (!client->Bind(&MarbleVirtualComms::CommsCallback, this)) {
    // If binding fails, it's probably because a previous client for this address is still
    // registered with the broker.  Calling destructor should unregister and allow us to retry.
    // This works about half the time, and may cause crashes, but it's good enough.
    client->~CommsClient();
    client = std::make_unique<subt::CommsClient>(id, false);
    if (!client->Bind(&MarbleVirtualComms::CommsCallback, this)) {
      ROS_FATAL("marble_virtual_comms did not successfully bind to the CommsClient");
    }
  } else {
    ROS_INFO_STREAM(client->Host() << " registered");
  }

  sendpre = "/" + id + "/mesh_comm/send/";
  recvpre = "/" + id + "/mesh_comm/recv/";
  createPeerSrv = private_nh.advertiseService("create_peer", &MarbleVirtualComms::createPeerService, this);
  missing_timer = nh.createTimer(ros::Duration(timer), &MarbleVirtualComms::missingTimer, this);
  // neighbor_timer = nh.createTimer(ros::Duration(10.0), &MarbleVirtualComms::neighborsPrintTimer, this);
  // artifact_timer = nh.createTimer(ros::Duration(10.0), &MarbleVirtualComms::artifactTestTimer, this);

  // Create an "all" peer for storing broadcast data
  MVCPeer peer = MVCPeer();
  peer.init("all", *this);
  peers.insert(std::make_pair("all", peer));
}

MarbleVirtualComms::~MarbleVirtualComms() {
  client->~CommsClient();
}

template<typename M>
void MarbleVirtualComms::sendMsg(const M& msg, uint8_t msg_type, std::string remote) {
  // Receive data from the local agent, generate a string, and send to the remote peer
  uint32_t serial_size = ros::serialization::serializationLength(*msg);
  boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
  ros::serialization::OStream stream(buffer.get(), serial_size);
  ros::serialization::serialize(stream, *msg);

  // Figure out how many packets we'll need
  uint16_t total_packets = (serial_size + PACKET_SIZE - 1) / PACKET_SIZE;

  auto& peer = peers[remote];
  MVCPeerTopic* peerTopic = &peer.topics[msg_type];
  MVCPeerData* peerData = &peerTopic->sbuffer[++peerTopic->seq];
  peerData->packets.resize(total_packets);
  peerData->time = ros::Time::now();

  // Convert the stream into a vector of strings
  for (auto i = 0; i < total_packets; ++i) {
    // Add the message type (0 == ma_data, 1 == dm_req, 2 == dm_resp)
    peerData->packets[i].push_back(msg_type & 0xFF);
    // Add the sequence number
    peerData->packets[i].push_back((peerTopic->seq >> 8) & 0xFF);
    peerData->packets[i].push_back(peerTopic->seq & 0xFF);
    // Add the packet number
    peerData->packets[i].push_back((i >> 8) & 0xFF);
    peerData->packets[i].push_back(i & 0xFF);
    // Add the total packets to look for
    peerData->packets[i].push_back((total_packets >> 8) & 0xFF);
    peerData->packets[i].push_back(total_packets & 0xFF);

    // Get the portion of the buffer for this string
    for (auto j = 0; j < PACKET_SIZE; ++j) {
      // Make sure we don't go beyond the buffer size
      uint32_t idx = i * PACKET_SIZE + j;
      if (idx < serial_size) {
        peerData->packets[i].push_back(buffer[idx]);
      }
    }
  }

  if (remote == "all") {
    // Send the string to each neighbor
    for (auto apeer : peers) {
      if (apeer.first != "all") {
        for (auto i = 0; i < total_packets; ++i) {
          client->SendTo(peerData->packets[i], apeer.first);
        }
      }
    }
  } else {
    // Send the string packets to this peer
    for (auto i = 0; i < total_packets; ++i) {
      client->SendTo(peerData->packets[i], peer.name);
    }
  }
}

void MarbleVirtualComms::receiveMsg(std::string data, std::string remote) {
  // Get the header information
  uint8_t  msg_type =      (unsigned char)(data[0]);
  uint16_t seq =          ((unsigned char)(data[1]) << 8) +
                           (unsigned char)(data[2]);
  uint16_t packet_num =   ((unsigned char)(data[3]) << 8) +
                           (unsigned char)(data[4]);
  uint16_t packet_total = ((unsigned char)(data[5]) << 8) +
                           (unsigned char)(data[6]);

  auto& peer = peers[remote];
  MVCPeerTopic* peerTopic = &peer.topics[msg_type];
  MVCPeerData* peerData = &peerTopic->rbuffer[seq];

  // Initialize this sequence if it's new
  if (peerData->recvd_packets == 0) {
    peerData->init(seq, packet_total);
  }

  // Save the data to the sequence's buffer if it wasn't previously received
  if (peerData->packets[packet_num] == "") {
    peerData->addPacket(data, packet_num);
  }

  // If we've received all of packets for this sequence, process the full string
  if (peerData->recvd_packets == packet_total) {
    uint32_t serial_size = peerData->data_length;
    boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);

    // Add all of the strings from all of the packets into the buffer
    for (auto i = 0; i < packet_total; ++i) {
      for (auto j = 0; j < peerData->packets[i].size() - HEADER_SIZE; ++j) {
        buffer[i * PACKET_SIZE + j] = (unsigned char)(peerData->packets[i][j + HEADER_SIZE]);
      }
    }

    ros::serialization::IStream stream(buffer.get(), serial_size);
    // Publish the stream to the appropriate local topic
    if (msg_type == 0)
      publishStreamMAData(stream, peer.topics[msg_type].pub);
    else if (msg_type == 1)
      publishStreamDMReq(stream, peer.topics[msg_type].pub);
    else if (msg_type == 2)
      publishStreamDMResp(stream, peer.topics[msg_type].pub);

    // Cleanup our buffer since this sequence is complete
    std::map<uint16_t, MVCPeerData>::iterator it = peerTopic->rbuffer.begin();
    while (it != peerTopic->rbuffer.end()) {
      if (it->first <= seq) {
        it = peerTopic->rbuffer.erase(it);
      } else {
        ++it;
      }
    }
  } else {
    // If the message isn't complete, check if we need to request a missing packet
    // If the packet isn't the next in sequence, we're missing one or more, so request them
    if (packet_num > peerData->last_packet + 1) {
      requestMissing(seq, msg_type, peerData->last_packet, packet_num, peer.name);
    }

    // Record the last packet received
    if (packet_num > peerData->last_packet) {
      peerData->last_packet = packet_num;
    }
  }
}

void MarbleVirtualComms::resendMissing(std::string data, std::string remote) {
  // Get the request information
  uint8_t  msg_type =      (unsigned char)(data[1]);
  uint16_t seq =          ((unsigned char)(data[2]) << 8) +
                           (unsigned char)(data[3]);
  uint16_t first_packet = ((unsigned char)(data[4]) << 8) +
                           (unsigned char)(data[5]);
  uint16_t packet_total = ((unsigned char)(data[6]) << 8) +
                           (unsigned char)(data[7]);

  auto peer = peers[remote];
  if (msg_type == 0) {
    peer = peers["all"];
  }
  MVCPeerTopic* peerTopic = &peer.topics[msg_type];
  MVCPeerData* peerData = &peerTopic->sbuffer[seq];

  // Find the packets and send them again
  for (auto i = first_packet; i < first_packet + packet_total; ++i) {
    if (i < peerData->packets.size()) {
      std::string data = peerData->packets[i];
      client->SendTo(data, remote);
    }
  }
}

void MarbleVirtualComms::requestMissing(uint16_t seq, uint8_t msg_type, uint16_t last_packet, uint16_t packet_num, std::string name) {
  // Build a missing packet request and send it
  std::string request;
  uint16_t req = last_packet + 1;
  // Message type (3 = missing packets request)
  request.push_back(3 & 0xFF);
  // Message type of the missing packets
  request.push_back(msg_type & 0xFF);
  // Sequence of the missing packets
  request.push_back((seq >> 8) & 0xFF);
  request.push_back(seq & 0xFF);
  // First missing packet
  request.push_back((req >> 8) & 0xFF);
  request.push_back(req & 0xFF);
  // Total number of missing packets
  request.push_back(((packet_num - req) >> 8) & 0xFF);
  request.push_back((packet_num - req) & 0xFF);
  // Send the request
  client->SendTo(request, name);
}

void MarbleVirtualComms::missingTimer(const ros::TimerEvent& event) {
  // Find all missing packets and request them
  // Check each peer
  for (auto& peer : peers) {
    // And each topic
    for (auto& topic : peer.second.topics) {
      // And each sequence
      // Need to use the iterator so we can delete expired topics
      std::map<uint16_t, MVCPeerData>::iterator it = topic.rbuffer.begin();
      while (it != topic.rbuffer.end()) {
        // If the message is expired, delete from the buffer
        if (it->second.time + ros::Duration(timeout) < ros::Time::now()) {
          it = topic.rbuffer.erase(it);
        } else {
          MVCPeerData* checkData = &it->second;
          uint16_t last_packet = checkData->last_packet;
          uint16_t packet_total = checkData->packet_total;
          uint16_t req;
          for (uint16_t i = 0; i < packet_total; ++i) {
            // Identify missing packet
            if (checkData->packets[i] == "") {
              req = i - 1;
              // Find consecutive missing packets
              while ((checkData->packets[i] == "") && (i < packet_total))
                i++;
              // Make the request
              requestMissing(checkData->seq, topic.msg_type, req, i, peer.first);
            }
          }
          ++it;
        }
      }

      // Cleanup the send buffer as well
      it = topic.sbuffer.begin();
      while (it != topic.sbuffer.end()) {
        if (it->second.time + ros::Duration(timeout) < ros::Time::now()) {
          it = topic.sbuffer.erase(it);
        } else {
          ++it;
        }
      }
    }
  }
}

void MarbleVirtualComms::CommsCallback(const std::string& srcAddress, const std::string& dstAddress, const uint32_t dstPort, const std::string& data) {
  // Receive a string from a neighbor

  if (srcAddress == std::string("base_station")) {
    // Process artifact submission response
    subt::msgs::ArtifactScore res;
    if (!res.ParseFromString(data)) {
      ROS_ERROR("error parsing message");
    }

    int score = res.score_change();
    geometry_msgs::Point location;
    location.x = res.artifact().pose().position().x();
    location.y = res.artifact().pose().position().y();
    location.z = res.artifact().pose().position().z();

    ROS_INFO_STREAM("Artifact " << location.x << location.y << location.z <<
                    " received at base station with score " << score);
  } else {
    // Make sure this peer exists, and add if it doesn't
    if (peers.find(srcAddress) == peers.end())
      createPeer(srcAddress);

    // Process messages from other agents
    if ((unsigned char)(data[0]) == 3) {
      // msg_type = 3 is a missing data request
      resendMissing(data, srcAddress);
    } else {
      // Regular messages
      receiveMsg(data, srcAddress);
    }
  }
}

// Callbacks for local message subscribers
void MarbleVirtualComms::maDataCallback(const marble_multi_agent::AgentMsgConstPtr& msg) {
  sendMsg(msg, 0, "all");
}

void MarbleVirtualComms::dmReqCallback(const marble_multi_agent::DMReqArrayConstPtr& msg, std::string remote) {
  sendMsg(msg, 1, remote);
}

void MarbleVirtualComms::dmRespCallback(const marble_multi_agent::DMRespArrayConstPtr& msg, std::string remote) {
  sendMsg(msg, 2, remote);
}

// Publishers for local messages after receiving from a peer
void MarbleVirtualComms::publishStreamMAData(ros::serialization::IStream stream, ros::Publisher pub) {
  marble_multi_agent::AgentMsg msg;
  ros::serialization::deserialize(stream, msg);
  pub.publish(msg);
}

void MarbleVirtualComms::publishStreamDMReq(ros::serialization::IStream stream, ros::Publisher pub) {
  marble_multi_agent::DMReqArray msg;
  ros::serialization::deserialize(stream, msg);
  pub.publish(msg);
}

void MarbleVirtualComms::publishStreamDMResp(ros::serialization::IStream stream, ros::Publisher pub) {
  marble_multi_agent::DMRespArray msg;
  ros::serialization::deserialize(stream, msg);
  pub.publish(msg);
}

bool MarbleVirtualComms::createPeerService(marble_virtual_comms::CreatePeer::Request& req, marble_virtual_comms::CreatePeer::Response& resp) {
  return createPeer(req.remote);
}

bool MarbleVirtualComms::createPeer(std::string remote) {
  if (peers.find(remote) != peers.end()) {
    ROS_WARN_STREAM(id << " is already connected to " << remote);
    return false;
  }

  MVCPeer peer = MVCPeer();
  peer.init(remote, *this);
  peers.insert(std::make_pair(remote, peer));

  return true;
}

void MarbleVirtualComms::neighborsPrintTimer(const ros::TimerEvent& event) {
  subt::CommsClient::Neighbor_M neighbors = client->Neighbors();
  subt::CommsClient::Neighbor_M::iterator it = neighbors.begin();
  while (it != neighbors.end()) {
    ROS_INFO_STREAM(client->Host() << " has neighbor " << it->first <<
                    " at time " << it->second.first << " with signal strength " <<
                    it->second.second << " dBm");
    it++;
  }
}

void MarbleVirtualComms::artifactTestTimer(const ros::TimerEvent& event) {
  ignition::msgs::Pose pose;
  pose.mutable_position()->set_x(70.0);
  pose.mutable_position()->set_y(-3.9);
  pose.mutable_position()->set_z(0.2);

  subt::msgs::Artifact artifact;
  artifact.set_type(static_cast<uint32_t>(subt::ArtifactType::TYPE_BACKPACK));
  artifact.mutable_pose()->CopyFrom(pose);

  std::string serializedData;
  if (!artifact.SerializeToString(&serializedData)) {
    ROS_ERROR_STREAM("Error serializing message\n" << artifact.DebugString());
  }

  client->SendTo(serializedData, subt::kBaseStationName);
}

}
