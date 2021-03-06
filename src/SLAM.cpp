//
// Created by quantum on 3/18/22.
//

#include "SLAM.h"
#include "Log.h"

SLAM::SLAM()
{
   using namespace gtsam;
   _fgh.AddPriorPoseFactor(_currentPoseId, Pose3::identity());
   _fgh.SetPoseInitialValue(_currentPoseId, Pose3::identity());
}

void SLAM::LoadPlanarSLAM(const std::string& filepath)
{
}

void SLAM::PoseUpdate(const RigidBodyTransform& odometry, int poseId)
{
   using namespace gtsam;

   /*TODO: Create Global Initial Value and Create Local Measurement Factor. */
   _sensorToMapTransform.MultiplyRight(odometry);

   Eigen::Quaterniond quaternion = _sensorToMapTransform.GetQuaternion();
   Eigen::Vector3d position = _sensorToMapTransform.GetTranslation();
   MS_DEBUG("Sensor To Map Pose ({}): {} {} {} {} {} {} {}", poseId, position.x(),
                 position.y(), position.z(), quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w());

   quaternion = odometry.GetQuaternion();
   position = odometry.GetTranslation();
   MS_DEBUG("Odometry ({}): {} {} {} {} {} {} {}", poseId, position.x(),
                 position.y(), position.z(), quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w());

   _fgh.AddOdometryFactor(gtsam::Pose3(odometry.GetMatrix()), poseId);
   _fgh.SetPoseInitialValue(poseId, gtsam::Pose3(_sensorToMapTransform.GetInverse().GetMatrix()));
   _poseKeys.push_back(poseId);
}

void SLAM::LandmarkUpdate(const std::unordered_map<int, Plane3D>& planes, int poseId)
{
   using namespace gtsam;

   for(auto plane : planes)
   {
      MS_DEBUG("Local Plane: {}", plane.second.GetString());
      _fgh.AddOrientedPlaneFactor(plane.second.GetParams(), plane.second.GetID(), poseId);
      Plane3D planeInMapFrame = plane.second.GetTransformed(_sensorToMapTransform);
      MS_DEBUG("Transformed Plane: {}", planeInMapFrame.GetString());
      _fgh.SetOrientedPlaneInitialValue(plane.second.GetID(), gtsam::OrientedPlane3(planeInMapFrame.GetParams()));
      _planarMap.InsertPlane(planeInMapFrame, plane.second.GetID());
      _landmarkKeys.push_back(plane.second.GetID());
   }

//   _fgh.optimize();
   _fgh.OptimizeISAM2(8);
   _fgh.ClearISAM2();



//   _fgh.GetResults().print("Result Planes");
}

void SLAM::GetResultPlanes(PlaneSet3D& resultSet)
{
   auto result = _fgh.GetResults();
   for (auto plane : _planarMap.GetPlanes())
   {
      auto params = result.at<gtsam::OrientedPlane3>(gtsam::Symbol('l', plane.second.GetID())).planeCoefficients();
      resultSet.InsertPlane(Plane3D(params.x(), params.y(), params.z(), params.w(), plane.second.GetID()), plane.second.GetID());
   }
}

void SLAM::GetResultPose(RigidBodyTransform& transform, int poseId)
{
   transform.SetMatrix(_fgh.GetResults().at<gtsam::Pose3>(gtsam::Symbol('x', poseId)).matrix());
   transform.SetID(poseId);
}

void SLAM::GetResultPoses(std::vector<RigidBodyTransform>& poses)
{
   for (int id : _poseKeys)
   {
      RigidBodyTransform transform;
      transform.SetMatrix(_fgh.GetResults().at<gtsam::Pose3>(gtsam::Symbol('x', id)).matrix());
      transform.SetID(id);
      poses.push_back(std::move(transform));
   }
}

void SLAM::SLAMTest()
{
   using namespace gtsam;

   Pose3 init_pose(Rot3::Ypr(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));
   PoseUpdate(RigidBodyTransform(init_pose.matrix()), 1);

   std::unordered_map<int, Plane3D> planes;
   planes[0] = (Plane3D(0.8, 0.1, 0.1, 2.9, 0, 0, 0));
   planes[1] = (Plane3D(0.1, 0.04, 1.1, 0, 0, 3.1, 1));
   LandmarkUpdate(planes, 0);

   Pose3 odometry(Rot3::Ypr(0.0, 0.0, 0.0), Point3(1.0, 0.0, 0.0));
   PoseUpdate(RigidBodyTransform(odometry.matrix()), 2);

   planes.clear();
   planes[0] = (Plane3D(0.8, 0.1, 0.1, 2.1, 0, 0, 0));
   planes[1] = (Plane3D(0.1, 0.04, 1.1, -1.1, 0.05, 2.9, 1));
   LandmarkUpdate(planes, 1);

}

