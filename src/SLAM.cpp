//
// Created by quantum on 3/18/22.
//

#include "SLAM.h"

SLAM::SLAM()
{
   using namespace gtsam;
   _fgh.AddPriorPoseFactor(Pose3::identity());
   _fgh.SetPoseInitialValue(_currentPoseId, Pose3::identity());
}

void SLAM::LoadPlanarSLAM(const std::string& filepath)
{
}

void SLAM::PoseUpdate(const RigidBodyTransform& odometry, int poseId)
{
   using namespace gtsam;

   /*TODO: Create Global Initial Value and Create Local Measurement Factor. */
   _sensorPoseMapFrame.MultiplyRight(RigidBodyTransform(odometry.GetMatrix()));
   _fgh.AddOdometryFactor(gtsam::Pose3(odometry.GetMatrix()), poseId);
   _fgh.SetPoseInitialValue(poseId, gtsam::Pose3(odometry.GetMatrix()));
}

void SLAM::LandmarkUpdate(const std::vector<Plane3D>& planes, int poseId)
{
   using namespace gtsam;

   for(auto plane : planes)
   {
      _fgh.AddOrientedPlaneFactor(plane.GetParams(), plane.GetID(), poseId);
      Plane3D planeInWorldFrame = plane.GetTransformed(_sensorPoseMapFrame);
      _fgh.SetOrientedPlaneInitialValue(plane.GetID(), gtsam::OrientedPlane3(planeInWorldFrame.GetParams()));
   }

   _fgh.optimize();
   _fgh.GetResults().print("Result Planes");
}

void SLAM::SLAMTest()
{
   using namespace gtsam;

   Pose3 init_pose(Rot3::Ypr(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));
   PoseUpdate(RigidBodyTransform(init_pose.matrix()), 1);

   std::vector<Plane3D> planes;
   planes.push_back(Plane3D(0.8, 0.1, 0.1, 2.9, 0, 0, 0));
   planes.push_back(Plane3D(0.1, 0.04, 1.1, 0, 0, 3.1, 1));
   LandmarkUpdate(planes, 0);

   Pose3 odometry(Rot3::Ypr(0.0, 0.0, 0.0), Point3(1.0, 0.0, 0.0));
   PoseUpdate(RigidBodyTransform(odometry.matrix()), 2);

   planes.clear();
   planes.push_back(Plane3D(0.8, 0.1, 0.1, 2.1, 0, 0, 0));
   planes.push_back(Plane3D(0.1, 0.04, 1.1, -1.1, 0.05, 2.9, 1));
   LandmarkUpdate(planes, 1);

}