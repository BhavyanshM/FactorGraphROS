//
// Created by quantum on 3/18/22.
//

#ifndef FACTOR_GRAPH_NODE_SLAM_H
#define FACTOR_GRAPH_NODE_SLAM_H

#include "FactorGraphHandler.h"

class SLAM
{
   public:
      SLAM ();
      void LoadPlanarSLAM(const std::string& filepath);

      FactorGraphHandler& GetFactorGraphHandler() {return _fgh;}

   private:
      FactorGraphHandler _fgh;
};

#endif //FACTOR_GRAPH_NODE_SLAM_H
