//
// Created by isr-lab on 1/10/22.
//

#include "FactorGraphNodeLayer.h"
#include "Scene/Mesh/MeshTools.h"
#include "MeshGenerator.h"

namespace Clay
{
   FactorGraphNodeLayer::FactorGraphNodeLayer(int argc, char **argv) : ApplicationLayer(argc, argv)
   {
      _network = new NetworkManager(appState);
      _network->InitNode(argc, argv, appState);

      firstCloud = std::make_shared<PointCloud>(glm::vec4(0.7f, 0.4f, 0.5f, 1.0f), _rootModel);
      _models.emplace_back(std::dynamic_pointer_cast<Model>(firstCloud));

   }

   void FactorGraphNodeLayer::MapsenseUpdate()
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

            Eigen::Quaterniond quaternion = transform.GetQuaternion();
            Eigen::Vector3d position = transform.GetTranslation();
            CLAY_LOG_INFO("Result Pose ({}): {} {} {} {} {} {} {}", _network->GetPlaneSets()[0].GetID(), position.x(),
                          position.y(), position.z(), quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w());

            _network->GetPlaneSets().pop_front();
            _network->PublishPlaneSet(resultSet);
            _network->PublishPose(transform);

//            mesher.GeneratePoseMesh(transform.GetMatrix().cast<float>(), nullptr)
            
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

   void FactorGraphNodeLayer::ImGuiUpdate(ApplicationState& appState)
   {
      if(ImGui::BeginTabItem("Factor Graph"))
      {
         if (ImGui::Button("Update"))
         {
            _slam.LoadPlanarSLAM("/home/quantum/Workspace/Volume/catkin_ws/src/MapSenseROS/Extras/");
         }
         ImGui::EndTabItem();
      }

   }
}

