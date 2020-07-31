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

namespace marble_virtual_comms {
class MarbleVirtualComms {

public:
  MarbleVirtualComms(const ros::NodeHandle private_nh_ = ros::NodeHandle("~"), const ros::NodeHandle &nh_ = ros::NodeHandle());
  virtual ~MarbleVirtualComms();

  virtual void CommsCallback(const std::string& srcAddress, const std::string& dstAddress, const uint32_t dstPort, const std::string& data);
  virtual void maDataCallback(const marble_multi_agent::AgentMsgConstPtr& msg);
  virtual void dmReqCallback(const marble_multi_agent::DMReqArrayConstPtr& msg);
  virtual void dmRespCallback(const marble_multi_agent::DMRespArrayConstPtr& msg);

protected:
  bool createPeerService(marble_virtual_comms::CreatePeer::Request& req, marble_virtual_comms::CreatePeer::Response& resp);
  void neighborsPrintTimer(const ros::TimerEvent& event);
  void artifactTestTimer(const ros::TimerEvent& event);

  ros::NodeHandle nh;
  ros::NodeHandle private_nh;
  ros::ServiceServer createPeerSrv;
  ros::Timer neighbor_timer;
  ros::Timer artifact_timer;

  std::string id;
  std::unique_ptr<subt::CommsClient> client;
  std::string sendpre;
  std::string recvpre;

  ros::Subscriber maDataSub;
  std::map<std::string, ros::Publisher> ma_data;
  std::map<std::string, std::pair<ros::Subscriber, ros::Publisher>> dm_req;
  std::map<std::string, std::pair<ros::Subscriber, ros::Publisher>> dm_resp;
};
}

#endif
