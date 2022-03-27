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
   resultPlanePub = rosNode->advertise<sensor_msgs::PointCloud2>("/slam/output/planes", 3);
   slamPosePub = rosNode->advertise<geometry_msgs::PoseStamped>("/slam/output/pose", 3);

   subMapSenseParams = rosNode->subscribe("/map/config", 8, &NetworkManager::MapsenseParamsCallback, this);
   rawPoseSub = rosNode->subscribe("/mapsense/slam/pose", 8, &NetworkManager::InputPoseCallback, this);
   rawPlaneSub = rosNode->subscribe("/slam/input/planes", 8, &NetworkManager::InputPlaneCallback, this);

   CLAY_LOG_INFO("Started ROS Node");
}

void NetworkManager::InputPlaneCallback(const sensor_msgs::PointCloud2ConstPtr& planes)
{
   int totalBytes = planes->width * planes->height * planes->point_step;
   std::vector<float> points(totalBytes / sizeof(float));
   memcpy(points.data(), planes->data.data(), totalBytes);

   PlaneSet3D planeSet;
   planeSet.SetID((int)planes->header.seq + 1);
   for(int i = 0; i<planes->width * planes->height; i++)
   {
      planeSet.InsertPlane(Plane3D(points[i*7 ] , points[i*7 + 1], points[i*7 + 2], points[i*7 + 3],
                                points[i*7 + 4], points[i*7 + 5], (int) points[i*7 + 6]), (int) points[i*7 + 6]);
   }
   _planeSets.push_back(planeSet);
   _planesAvailable = true;
}

void NetworkManager::InputPoseCallback(const geometry_msgs::PoseStamped pose)
{
   inputPoseMsg = pose;
   RigidBodyTransform transform;
   transform.SetQuaternionAndTranslation(Eigen::Quaterniond(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z),
                                         Eigen::Vector3d(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z));
   transform.SetID((int)pose.header.seq + 1);
   _poses.push_back(std::move(transform));
   paramsAvailable = true;
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

void NetworkManager::PublishPose(RigidBodyTransform worldToSensorTransform)
{
   Eigen::Quaterniond quaternion = worldToSensorTransform.GetQuaternion();
   Eigen::Vector3d position = worldToSensorTransform.GetTranslation();

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

void NetworkManager::PublishPlaneSet(const PlaneSet3D& set) const
{
   sensor_msgs::PointCloud2 planeSet;
   planeSet.header.seq = set.GetID();
   planeSet.width = set.GetPlanes().size();
   planeSet.height = 1;
   planeSet.row_step = 2;
   planeSet.point_step = 4 * 5;

   std::vector<float> points;

   for (auto plane : set.GetPlanes())
   {
      //      points.push_back((float)poseId);
      points.push_back(plane.second.GetParams().x());
      points.push_back(plane.second.GetParams().y());
      points.push_back(plane.second.GetParams().z());
      points.push_back(plane.second.GetParams().w());
      points.push_back((float)plane.second.GetID());
      CLAY_LOG_INFO("Publishing Plane ID: {}", plane.second.GetID());
   }

   std::vector<unsigned char> data(points.size() * sizeof(float));
   memcpy(data.data(), points.data(), data.size());

   planeSet.data = data;

   CLAY_LOG_INFO("Data: {} Points:{}", data.size(), points.size());

   resultPlanePub.publish(planeSet);
}

