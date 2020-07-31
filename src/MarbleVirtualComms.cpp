#include <MarbleVirtualComms.h>

namespace marble_virtual_comms {

MarbleVirtualComms::MarbleVirtualComms(const ros::NodeHandle private_nh_, const ros::NodeHandle &nh_) : nh(nh_), private_nh(private_nh_) {

  private_nh.param("vehicle", id, std::string("Base"));

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
  maDataSub = private_nh.subscribe(sendpre + "ma_data", 100, &MarbleVirtualComms::maDataCallback, this);
  createPeerSrv = private_nh.advertiseService("create_peer", &MarbleVirtualComms::createPeerService, this);
  neighbor_timer = nh.createTimer(ros::Duration(10.0), &MarbleVirtualComms::neighborsPrintTimer, this);
  artifact_timer = nh.createTimer(ros::Duration(10.0), &MarbleVirtualComms::artifactTestTimer, this);
}

MarbleVirtualComms::~MarbleVirtualComms(){
  client->~CommsClient();
}

void MarbleVirtualComms::CommsCallback(const std::string& srcAddress, const std::string& dstAddress, const uint32_t dstPort, const std::string& data) {

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
    // Process messages from other agents
    ROS_INFO_STREAM("Received message from " << srcAddress.data());
  }
}

void MarbleVirtualComms::maDataCallback(const marble_multi_agent::AgentMsgConstPtr& msg) {
  ROS_INFO("Received ma_data from %s", id.data());
}

void MarbleVirtualComms::dmReqCallback(const marble_multi_agent::DMReqArrayConstPtr& msg) {
  ROS_INFO("Received dm_req from %s", id.data());
}

void MarbleVirtualComms::dmRespCallback(const marble_multi_agent::DMRespArrayConstPtr& msg) {
  ROS_INFO("Received dm_resp from %s", id.data());
}

bool MarbleVirtualComms::createPeerService(marble_virtual_comms::CreatePeer::Request& req, marble_virtual_comms::CreatePeer::Response& resp) {
  std::string remote = req.remote;

  if (dm_req.find(remote) != dm_req.end()) {
    ROS_WARN_STREAM(id << " is already connected to " << remote);
    return true;
  }

  ros::NodeHandle pnh("~");
  // Create the ma_data publisher
  ros::Publisher pub = pnh.advertise<marble_multi_agent::AgentMsg>(recvpre + remote + "/ma_data", 100);
  ma_data.insert(std::make_pair(remote, pub));

  // Create pair for DM Requests
  ros::Subscriber dm_req_sub = pnh.subscribe(sendpre + remote + "/dm_request", 100, &MarbleVirtualComms::dmReqCallback, this);
  ros::Publisher dm_req_pub = pnh.advertise<marble_multi_agent::DMReqArray>(recvpre + remote + "/dm_request", 100);
  dm_req.insert(std::make_pair(remote, std::make_pair(dm_req_sub, dm_req_pub)));

  // Create pair for DM Responses
  ros::Subscriber dm_resp_sub = pnh.subscribe(sendpre + remote + "/dm_response", 100, &MarbleVirtualComms::dmRespCallback, this);
  ros::Publisher dm_resp_pub = pnh.advertise<marble_multi_agent::DMRespArray>(recvpre + remote + "/dm_response", 100);
  dm_resp.insert(std::make_pair(remote, std::make_pair(dm_resp_sub, dm_resp_pub)));

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
