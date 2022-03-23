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

