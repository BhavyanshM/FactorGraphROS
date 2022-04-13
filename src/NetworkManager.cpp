#include "NetworkManager.h"
#include "Log.h"

NetworkManager::NetworkManager(ApplicationState& app)
{
}

void NetworkManager::SpinNode()
{
   MS_DEBUG("SpinOnce");
   ros::spinOnce();
   ros::Duration(0.1).sleep();
}

void NetworkManager::InitNode(int argc, char **argv, ApplicationState& app)
{
   MS_INFO("Starting ROS Node");

   ros::init(argc, argv, "FactorGraph");
   rosNode = new ros::NodeHandle();

   // ROSTopic Publishers
   planarRegionPub = rosNode->advertise<map_sense::RawGPUPlanarRegionList>("/mapsense/planar_regions", 3);
   resultPlanePub = rosNode->advertise<sensor_msgs::PointCloud2>("/slam/output/planes", 3);
   slamPosePub = rosNode->advertise<sensor_msgs::PointCloud2>("/slam/output/pose", 3);

   subMapSenseParams = rosNode->subscribe("/map/config", 8, &NetworkManager::MapsenseParamsCallback, this);
   rawPoseSub = rosNode->subscribe("/mapsense/slam/pose", 8, &NetworkManager::InputPoseCallback, this);
   rawPlaneSub = rosNode->subscribe("/slam/input/planes", 8, &NetworkManager::InputPlaneCallback, this);

   MS_INFO("Started ROS Node");
}

void NetworkManager::InputPlaneCallback(const sensor_msgs::PointCloud2ConstPtr& planes)
{
   int totalBytes = planes->width * planes->height * planes->point_step;
   std::vector<float> points(totalBytes / sizeof(float));
   memcpy(points.data(), planes->data.data(), totalBytes);

   PlaneSet3D planeSet;
   planeSet.SetID((int)points[0]);
   for(int i = 0; i<planes->width * planes->height; i++)
   {
      planeSet.InsertPlane(Plane3D(points[i*8 + 1], points[i*8 + 2], points[i*8 + 3],
                                points[i*8 + 4], points[i*8 + 5], points[i*8 + 6], (int) points[i*8+7]), (int) points[i*8 + 7]);
      MS_DEBUG("Received Plane({}): {} {} {} {} {} {}", (int) points[i*8 + 7], points[i*8 + 1], points[i*8 + 2], points[i*8 + 3],
                    points[i*8 + 4], points[i*8 + 5], points[i*8 + 6]);
   }
   _planeSets.push_back(std::move(planeSet));
   _planesAvailable = true;
}

void NetworkManager::InputPoseCallback(const sensor_msgs::PointCloud2ConstPtr& poses)
{

   int totalBytes = poses->width * poses->height * poses->point_step;
   std::vector<float> points(totalBytes / sizeof(float));
   memcpy(points.data(), poses->data.data(), totalBytes);

   for(int i = 0; i<poses->width * poses->height; i++)
   {
      RigidBodyTransform transform;
      transform.SetQuaternionAndTranslation(Eigen::Quaterniond(points[i*8+6], points[i*8 + 3],points[i*8 + 4], points[i*8 + 5]),
                                            Eigen::Vector3d(points[i*8 ] , points[i*8 + 1], points[i*8 + 2]));
      transform.SetID((int) points[i*8+7]);
      MS_DEBUG("Received Pose ({}): {} {} {} {} {} {} {}", points[i*8+7],
                    points[i*8 ] , points[i*8 + 1], points[i*8 + 2], points[i*8 + 3],points[i*8 + 4], points[i*8 + 5],points[i*8 + 6]);
      _poses.push_back(std::move(transform));
      paramsAvailable = true;
   }
}

void NetworkManager::MapsenseParamsCallback(const map_sense::MapsenseConfiguration msg)
{
   paramsMessage = msg;
   paramsAvailable = true;
   MS_DEBUG("PARAMS CALLBACK");
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

void NetworkManager::PublishPoses(const std::vector<RigidBodyTransform>& transforms)
{
   sensor_msgs::PointCloud2 poseSet;
   poseSet.width = transforms.size();
   poseSet.height = 1;
   poseSet.row_step = 1;
   poseSet.point_step = 4 * 8;

   std::vector<float> points;

   for (auto tf : transforms)
   {
      Eigen::Quaterniond quaternion = tf.GetQuaternion();
      Eigen::Vector3d position = tf.GetTranslation();
      points.push_back(position.x());
      points.push_back(position.y());
      points.push_back(position.z());
      points.push_back(quaternion.x());
      points.push_back(quaternion.y());
      points.push_back(quaternion.z());
      points.push_back(quaternion.w());
      points.push_back(tf.GetID());
      MS_DEBUG("Publishing Pose ID: {}", id);
   }

   std::vector<unsigned char> data(points.size() * sizeof(float));
   memcpy(data.data(), points.data(), data.size());

   poseSet.data = data;

   MS_DEBUG("Data: {} Points:{}", data.size(), points.size());

   slamPosePub.publish(poseSet);
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
      MS_DEBUG("Publishing Plane ID: {}", plane.second.GetID());
   }

   std::vector<unsigned char> data(points.size() * sizeof(float));
   memcpy(data.data(), points.data(), data.size());

   planeSet.data = data;

   MS_DEBUG("Data: {} Points:{}", data.size(), points.size());

   resultPlanePub.publish(planeSet);
}

