//
// Created by quantum on 3/20/22.
//

#include "FactorGraphNodeHeadless.h"
#include "Core/Log.h"

FactorGraphNodeHeadless::FactorGraphNodeHeadless(int argc, char **argv)
{
   _network = new NetworkManager(appState);
   _network->InitNode(argc, argv, appState);
}

[[noreturn]] void FactorGraphNodeHeadless::Run()
{
   while(true)
   {
      Update();
   }
}

void FactorGraphNodeHeadless::Update()
{
   if(appState.ROS_ENABLED)
   {
      _network->SpinNode();
      _network->AcceptMapsenseConfiguration(appState);


      if(_network->GetPlaneSets().size() > 0)
      {
         CLAY_LOG_INFO("LandmarkUpdate: PlaneID: {}, Total PlanSets: {}", _network->GetPlaneSets()[0].GetID(), _network->GetPlaneSets().size());

         _slam.LandmarkUpdate(_network->GetPlaneSets()[0].GetPlanes(), _network->GetPlaneSets()[0].GetID());

         PlaneSet3D resultSet;
         _slam.GetResultPlanes(resultSet);

         RigidBodyTransform transform;
         _slam.GetResultPose(transform, _network->GetPlaneSets()[0].GetID());

         _network->GetPlaneSets().pop_front();
         _network->PublishPlaneSet(resultSet);
         _network->PublishPose(transform);

      }

      if(_network->GetPoses().size() > 0)
      {
         CLAY_LOG_INFO("PoseUpdate: PoseID: {} TotalPoses: {}", _network->GetPoses()[0].GetID(), _network->GetPoses().size());

         _slam.PoseUpdate(_network->GetPoses()[0], _network->GetPoses()[0].GetID() + 1);
         _network->GetPoses().pop_front();

//         for(int i = 0; i<_network->GetPoses().size(); i++)
//         {
//            printf("%d\t", _network->GetPoses()[i].GetID());
//         }
//         printf("\n");

      }
   }
}

int main(int argc, char** argv)
{
   FactorGraphNodeHeadless fg(argc, argv);
   fg.Run();
}
