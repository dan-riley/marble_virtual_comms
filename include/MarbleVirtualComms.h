#ifndef MARBLE_VIRTUAL_COMMSMARBLEVIRTUALCOMMS_H
#define MARBLE_VIRTUAL_COMMSMARBLEVIRTUALCOMMS_H

#include <ros/ros.h>
#include "boost/bind.hpp"
#include "boost/ref.hpp"
#include "geometry_msgs/Point.h"
#include "ignition/msgs.hh"
#include <std_srvs/SetBool.h>
#include "subt_communication_broker/subt_communication_client.h"
#include "subt_ign/CommonTypes.hh"
#include "subt_ign/protobuf/artifact.pb.h"
#include "marble_multi_agent/AgentMsg.h"
#include "marble_multi_agent/DMReqArray.h"
#include "marble_multi_agent/DMRespArray.h"
#include "marble_virtual_comms/CreatePeer.h"
#include "MVCPeer.h"

namespace marble_virtual_comms {

// PACKET_SIZE + HEADER_SIZE must be <= 1500
const uint16_t PACKET_SIZE = 1493;
const uint8_t HEADER_SIZE = 7;

class MarbleVirtualComms {

public:
  MarbleVirtualComms(const ros::NodeHandle private_nh_ = ros::NodeHandle("~"), const ros::NodeHandle &nh_ = ros::NodeHandle());
  virtual ~MarbleVirtualComms();

  virtual void CommsCallback(const std::string& srcAddress, const std::string& dstAddress, const uint32_t dstPort, const std::string& data);
  virtual void maDataCallback(const marble_multi_agent::AgentMsgConstPtr& msg);
  virtual void dmReqCallback(const marble_multi_agent::DMReqArrayConstPtr& msg, std::string remote);
  virtual void dmRespCallback(const marble_multi_agent::DMRespArrayConstPtr& msg, std::string remote);

  std::string sendpre;
  std::string recvpre;

protected:
  bool createPeerService(CreatePeer::Request& req, CreatePeer::Response& resp);
  bool createPeer(std::string remote);

  // Message handling
  template<typename M>
  void sendMsg(const M& msg, uint8_t msg_type, std::string remote);
  void receiveMsg(std::string data, std::string remote);
  void resendMissing(std::string data, std::string remote);
  void requestMissing(uint16_t seq, uint8_t msg_type, uint16_t last_packet, uint16_t packet_total, std::string name);
  // Republishers to local topics
  void publishStreamMAData(ros::serialization::IStream stream, ros::Publisher pub);
  void publishStreamDMReq(ros::serialization::IStream stream, ros::Publisher pub);
  void publishStreamDMResp(ros::serialization::IStream stream, ros::Publisher pub);
  // Timers
  void missingTimer(const ros::TimerEvent& event);
  void neighborsPrintTimer(const ros::TimerEvent& event);
  void artifactTestTimer(const ros::TimerEvent& event);

  ros::NodeHandle nh;
  ros::NodeHandle private_nh;
  ros::ServiceServer createPeerSrv;
  ros::Timer missing_timer;
  ros::Timer neighbor_timer;
  ros::Timer artifact_timer;

  std::string id;
  double timeout;
  double timer;
  std::unique_ptr<subt::CommsClient> client;
  std::map<std::string, MVCPeer> peers;
};
}

#endif
