
#include "FactorGraphHandler.h"

FactorGraphHandler::FactorGraphHandler()
{
   /* Set ISAM2 parameters here. */
   parameters.relinearizeThreshold = 0.01;
   parameters.relinearizeSkip = 1;
   this->isam = gtsam::ISAM2(parameters);

   gtsam::Vector6 odomVariance;
   odomVariance << 1e-2, 1e-2, 1e-2, 1e-1, 1e-1, 1e-1;
   createOdometryNoiseModel(odomVariance);

   gtsam::Vector3 lmVariance;
   lmVariance << 1e-2, 1e-2, 1e-2;
   createOrientedPlaneNoiseModel(lmVariance);

   gtsam::Vector6 priorVariance;
   priorVariance << 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4;
   priorNoise = gtsam::noiseModel::Diagonal::Variances(priorVariance);

   gtsam::Vector6 priorVariance2;
   priorVariance2 << 1e2, 1e2, 1e2, 1e2, 1e2, 1e2;
   priorNoise2 = gtsam::noiseModel::Diagonal::Variances(priorVariance2);


}

//void FactorGraphHandler::getPoses(std::vector<RigidBodyTransform>& poses)
//{
//   poses.clear();
//   for (int i = 1; i < this->getPoseId(); i++)
//   {
//      RigidBodyTransform mapToSensorTransform(this->getResults().at<gtsam::Pose3>(gtsam::Symbol('x', i)).matrix());
//      poses.emplace_back(mapToSensorTransform);
//   }
//}

void FactorGraphHandler::createOdometryNoiseModel(gtsam::Vector6 odomVariance)
{
   odometryNoise = gtsam::noiseModel::Diagonal::Variances(odomVariance);
}

void FactorGraphHandler::createOrientedPlaneNoiseModel(gtsam::Vector3 lmVariances)
{
   orientedPlaneNoise = gtsam::noiseModel::Diagonal::Variances(lmVariances);
}

int FactorGraphHandler::AddPriorPoseFactor(gtsam::Pose3 mean)
{
   LOG("AddPriorPoseFactor(x%d)\n", poseId);
   graph.add(gtsam::PriorFactor<gtsam::Pose3>(gtsam::Symbol('x', poseId), mean, priorNoise));
   return poseId;
}

int FactorGraphHandler::AddOdometryFactor(gtsam::Pose3 odometry)
{
   LOG("AddOdometryFactor(x%d -o- x%d)\n", poseId, poseId + 1);
   graph.add(gtsam::BetweenFactor<gtsam::Pose3>(gtsam::Symbol('x', poseId), gtsam::Symbol('x', poseId + 1), odometry, odometryNoise));
   poseId++;
   return poseId;
}

int FactorGraphHandler::AddOrientedPlaneFactor(gtsam::Vector4 lmMean, int lmId, int poseIndex)
{
   int landmarkId = (lmId != -1) ? lmId : newLandmarkId++;
   LOG("AddOrientedPlaneFactor(x%d -o- l%d)\n", poseIndex, landmarkId);
   graph.add(gtsam::OrientedPlane3Factor(lmMean, orientedPlaneNoise, gtsam::Symbol('x', poseIndex), gtsam::Symbol('l', landmarkId)));
   return landmarkId;
}

void FactorGraphHandler::SetPoseInitialValue(int index, gtsam::Pose3 value)
{
   LOG("SetPoseInitialValue(x%d)\n", index);
   if (structure.find('x' + std::to_string(index)) == structure.end())
   {
      structure.insert('x' + std::to_string(index));
      initial.insert(gtsam::Symbol('x', index), value);
   }
}

void FactorGraphHandler::SetOrientedPlaneInitialValue(int landmarkId, gtsam::OrientedPlane3 value)
{
   LOG("SetOrientedPlaneInitialValue(l%d)\n", landmarkId);
   if (!initial.exists(gtsam::Symbol('l', landmarkId)) && structure.find('l' + std::to_string(landmarkId)) == structure.end())
   {
      structure.insert('l' + std::to_string(landmarkId));
      initial.insert(gtsam::Symbol('l', landmarkId), value);
   }
}

void FactorGraphHandler::optimize()
{
   LOG("optimize()\n");
   result = gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();
}

void FactorGraphHandler::optimizeISAM2(uint8_t numberOfUpdates)
{
   LOG("optimizeISAM2()\n");
   isam.update(graph, initial);
   for (uint8_t i = 1; i < numberOfUpdates; i++)
   {
      isam.update();
   }
   result = isam.calculateEstimate();
   LOG("optimization complete()\n");
}

void FactorGraphHandler::clearISAM2()
{
   LOG("clearISAM2()\n");
   initial.clear();
   graph.resize(0);
}

gtsam::Values FactorGraphHandler::getResults()
{
   return result;
}

gtsam::Values FactorGraphHandler::getInitial()
{
   return initial;
}

gtsam::NonlinearFactorGraph FactorGraphHandler::getFactorGraph()
{
   return graph;
}

int FactorGraphHandler::getPoseId() const
{
   return poseId;
}

void FactorGraphHandler::incrementPoseId()
{
   poseId++;
}


void FactorGraphHandler::SLAMTest()
{
   using namespace gtsam;

   int currentPoseId = 1;

   Pose3 init_pose(Rot3::Ypr(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));
   currentPoseId = AddPriorPoseFactor(Pose3::identity());
   SetPoseInitialValue(currentPoseId, Pose3::identity());

   AddOrientedPlaneFactor(Vector4(1, 0, 0, -3), 0, currentPoseId);
   SetOrientedPlaneInitialValue(0, gtsam::OrientedPlane3(Vector4(0.8, 0.1, 0.1, -2.9)));

   AddOrientedPlaneFactor(Vector4(0, 0, 1, -3), 1, currentPoseId);
   SetOrientedPlaneInitialValue(1, gtsam::OrientedPlane3(Vector4(0.1, 0.04, 1.1, -2.8)));

   Pose3 odometry(Rot3::Ypr(0.0, 0.0, 0.0), Point3(1.0, 0.0, 0.0));
   currentPoseId = AddOdometryFactor(odometry);
   SetPoseInitialValue(currentPoseId, odometry);

   AddOrientedPlaneFactor(Vector4(1, 0, 0, -2), 0, currentPoseId);
   SetOrientedPlaneInitialValue(0, gtsam::OrientedPlane3(Vector4(0.8, 0.1, 0.1, -2.1)));

   AddOrientedPlaneFactor(Vector4(0, 0, 1, -3), 1, currentPoseId);
   SetOrientedPlaneInitialValue(1, gtsam::OrientedPlane3(Vector4(0.1, 0.04, 1.1, -2.2)));

   optimize();

   result.print("Result Planes");


//
//   AddPriorPoseFactor(Eigen::MatrixXd());
//
//   currentPoseId = AddOdometryFactor(Eigen::MatrixXd());
//   SetPoseInitialValue(currentPoseId, Eigen::MatrixXd());
//
//   /* Initialize poses and landmarks with map frame values. */
//   SetOrientedPlaneInitialValue(currentPoseId, gtsam::OrientedPlane3());
//
//   optimize();
//
//   clearISAM2();

      /* Load previous and current regions. Separated by SKIP_REGIONS. */
}
