#ifndef SRC_SENSORDATARECEIVER_H
#define SRC_SENSORDATARECEIVER_H

#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/PointCloud2.h"

#include "image_transport/image_transport.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "cv_bridge/cv_bridge.h"

#include "imgui.h"
#include "map_sense/RawGPUPlanarRegionList.h"
#include "map_sense/MapsenseConfiguration.h"

#include <ApplicationState.h>
#include "RigidBodyTransform.h"

typedef ros::master::TopicInfo TopicInfo;

class NetworkManager
{
   private:
      TopicInfo currentDataTopic, currentInfoTopic;

   public:
      map_sense::MapsenseConfiguration paramsMessage;
      geometry_msgs::PoseStamped inputPoseMsg;
      sensor_msgs::PointCloud2ConstPtr inputPlaneMsg;

      ros::NodeHandle *rosNode;

      ros::Subscriber subMapSenseParams;
      ros::Subscriber rawPoseSub;
      ros::Subscriber rawPlaneSub;

      ros::Publisher planarRegionPub;
      ros::Publisher slamPosePub;

      bool paramsAvailable = false;

      NetworkManager(ApplicationState& app);

      std::vector<TopicInfo> getROSTopicList();

      void InputPlaneCallback(const sensor_msgs::PointCloud2ConstPtr& planes);

      void InputPoseCallback(const geometry_msgs::PoseStamped pose);

      void MapsenseParamsCallback(const map_sense::MapsenseConfiguration msg);

      void InitNode(int argc, char **argv, ApplicationState& app);

      void SpinNode();

      void publishSLAMPose(RigidBodyTransform pose);

      void AcceptMapsenseConfiguration(ApplicationState& appState);

};

#endif //SRC_SENSORDATARECEIVER_H
