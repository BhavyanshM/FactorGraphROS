#include "NetworkManager.h"
#include "Core/Log.h"

NetworkManager::NetworkManager(ApplicationState& app)
{
}

void NetworkManager::SpinNode()
{
   ROS_DEBUG("SpinOnce");
   ros::spinOnce();
}

void NetworkManager::InitNode(int argc, char **argv, ApplicationState& app)
{
   CLAY_LOG_INFO("Starting ROS Node");

   ros::init(argc, argv, "FactorGraph");
   rosNode = new ros::NodeHandle();

   // ROSTopic Publishers
   planarRegionPub = rosNode->advertise<map_sense::RawGPUPlanarRegionList>("/mapsense/planar_regions", 3);
//   slamPosePub = rosNode->advertise<geometry_msgs::PoseStamped>("/mapsense/slam/pose", 3);

   subMapSenseParams = rosNode->subscribe("/map/config", 8, &NetworkManager::MapsenseParamsCallback, this);
   rawPoseSub = rosNode->subscribe("/mapsense/slam/pose", 8, &NetworkManager::InputPoseCallback, this);
   rawPlaneSub = rosNode->subscribe("/slam/input/planes", 8, &NetworkManager::InputPlaneCallback, this);

   CLAY_LOG_INFO("Started ROS Node");
}

void NetworkManager::InputPlaneCallback(const sensor_msgs::PointCloud2ConstPtr& planes)
{
   inputPlaneMsg = planes;

   int totalFloats = inputPlaneMsg->width * inputPlaneMsg->height * inputPlaneMsg->point_step / sizeof(float);

   std::vector<float> points(totalFloats);


   memcpy(points.data(), inputPlaneMsg->data.data(), totalFloats * sizeof(float));

   CLAY_LOG_INFO("Points: {} {} {} {} {} {} {} {}", points[0], points[1], points[2], points[3], points[4], points[5], points[6], points[7]);
}

void NetworkManager::InputPoseCallback(const geometry_msgs::PoseStamped pose)
{
   inputPoseMsg = pose;
   paramsAvailable = true;
   CLAY_LOG_INFO("Pose: {} {} {} {} {} {} {}", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
                 pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
}

void NetworkManager::MapsenseParamsCallback(const map_sense::MapsenseConfiguration msg)
{
   paramsMessage = msg;
   paramsAvailable = true;
   ROS_DEBUG("PARAMS CALLBACK");
}

void NetworkManager::AcceptMapsenseConfiguration(ApplicationState& appState)
{
   if (paramsAvailable)
   {
      paramsAvailable = false;
      appState.MERGE_DISTANCE_THRESHOLD = paramsMessage.mergeDistanceThreshold;
      appState.MERGE_ANGULAR_THRESHOLD = paramsMessage.mergeAngularThreshold;
      appState.GAUSSIAN_SIGMA = (int) paramsMessage.gaussianSigma;
      appState.GAUSSIAN_SIZE = (int) paramsMessage.gaussianSize;
      appState.REGION_GROWTH_FACTOR = paramsMessage.regionGrowthFactor;
   }
}

std::vector<TopicInfo> NetworkManager::getROSTopicList()
{
   ros::master::V_TopicInfo topic_infos;
   ros::master::getTopics(topic_infos);
   std::vector<TopicInfo> names;
   for (int i = 0; i < topic_infos.size(); i++)
   {
      names.emplace_back(topic_infos[i]);
   }
   return names;
}

void NetworkManager::publishSLAMPose(RigidBodyTransform worldToSensorTransform)
{
   Eigen::Quaterniond quaternion = worldToSensorTransform.getQuaternion();
   Eigen::Vector3d position = worldToSensorTransform.getTranslation();

   geometry_msgs::PoseStamped pose;
   pose.pose.position = geometry_msgs::Point();
   pose.pose.position.x = position.x();
   pose.pose.position.y = position.y();
   pose.pose.position.z = position.z();

   pose.pose.orientation = geometry_msgs::Quaternion();
   pose.pose.orientation.x = quaternion.x();
   pose.pose.orientation.y = quaternion.y();
   pose.pose.orientation.z = quaternion.z();
   pose.pose.orientation.w = quaternion.w();

   this->slamPosePub.publish(pose);
}

