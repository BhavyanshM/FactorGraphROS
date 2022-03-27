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
#include "Plane3D.h"
#include "deque"

typedef ros::master::TopicInfo TopicInfo;

class NetworkManager
{
   public:
      NetworkManager(ApplicationState& app);

      std::vector<TopicInfo> getROSTopicList();

      void InputPlaneCallback(const sensor_msgs::PointCloud2ConstPtr& planes);

      void InputPoseCallback(const geometry_msgs::PoseStamped pose);

      void MapsenseParamsCallback(const map_sense::MapsenseConfiguration msg);

      void InitNode(int argc, char **argv, ApplicationState& app);

      void SpinNode();

      void PublishPose(RigidBodyTransform worldToSensorTransform);

      void AcceptMapsenseConfiguration(ApplicationState& appState);

      std::deque<PlaneSet3D>& GetPlaneSets() {return _planeSets;}

      std::deque<RigidBodyTransform>& GetPoses() {return _poses;}

      bool GetPlanesAvailable() const { return _planesAvailable; }

      bool GetPoseAvailable() const { return _poseAvailable; }

      void SetPlanesAvailable(bool available) { _planesAvailable = available; }

      void PublishPlaneSet(const PlaneSet3D& set) const;

   private:
      map_sense::MapsenseConfiguration paramsMessage;
      geometry_msgs::PoseStamped inputPoseMsg;

      sensor_msgs::PointCloud2ConstPtr inputPlaneMsg;

      ros::NodeHandle *rosNode;
      ros::Subscriber subMapSenseParams;
      ros::Subscriber rawPoseSub;

      ros::Subscriber rawPlaneSub;
      ros::Publisher planarRegionPub;
      ros::Publisher resultPlanePub;

      ros::Publisher slamPosePub;

      TopicInfo currentDataTopic, currentInfoTopic;

      bool paramsAvailable = false;
      bool _planesAvailable = false;
      bool _poseAvailable = false;

      std::deque<PlaneSet3D> _planeSets;
      std::deque<RigidBodyTransform> _poses;

};

#endif //SRC_SENSORDATARECEIVER_H
