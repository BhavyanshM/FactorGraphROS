//
// Created by isr-lab on 1/10/22.
//

#ifndef MAP_SENSE_NETWORKEDTERRAINSLAMLAYER_H
#define MAP_SENSE_NETWORKEDTERRAINSLAMLAYER_H


#include "ApplicationLayer.h"
#include "SLAM.h"
#include "NetworkManager.h"

namespace Clay {
    class FactorGraphNodeLayer : public ApplicationLayer {

    public:
        FactorGraphNodeLayer(int argc, char **argv);
        void MapsenseUpdate() override;
        void ImGuiUpdate(ApplicationState& appState) override;

    private:
         NetworkManager* _network;
         Ref<PointCloud> firstCloud;
         SLAM _slam;

    };


}

#endif //MAP_SENSE_NETWORKEDTERRAINSLAMLAYER_H
