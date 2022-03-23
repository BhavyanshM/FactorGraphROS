//
// Created by quantum on 3/20/22.
//

#ifndef FACTOR_GRAPH_NODE_FACTORGRAPHNODEHEADLESS_H
#define FACTOR_GRAPH_NODE_FACTORGRAPHNODEHEADLESS_H

#include "string"
#include "vector"
#include "ApplicationState.h"
#include "NetworkManager.h"
#include "SLAM.h"

class FactorGraphNodeHeadless
{
   public:
      FactorGraphNodeHeadless(int argc, char** argv);

      [[noreturn]] void Run();

      void Update();

   private:
      ApplicationState appState;
      NetworkManager* _network;
      SLAM _slam;
};

#endif //FACTOR_GRAPH_NODE_FACTORGRAPHNODEHEADLESS_H
