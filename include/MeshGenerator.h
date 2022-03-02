#ifndef SRC_MESHGENERATOR_H
#define SRC_MESHGENERATOR_H

#include "Scene/Mesh/TriangleMesh.h"
#include "Core/Clay.h"

namespace Clay
{
   class MeshGenerator
   {
      public:

         MeshGenerator() {};

//         void GenerateMeshForRegions(std::vector<Ref<PlanarRegion>>& planarRegions, Ref<Model> parent);
//
//         void GenerateRegionLineMesh(shared_ptr<PlanarRegion>& planarRegion, Ref<TriangleMesh>& model);

         void InsertModel(Ref<TriangleMesh> model);

         const std::vector<Ref<Model>>& GetModels() const {return meshes;}

         //      void generateMatchLineMesh(vector<pair<int,int>> matches, vector<shared_ptr<PlanarRegion>> regions, vector<shared_ptr<PlanarRegion>> latestRegions, vector<Object3D *>& edges, Object3D* parent);
         //
         //      void generatePoseMesh(vector<RigidBodyTransform> poses, vector<Object3D*>& edges, Object3D* parent, int color, float scale = 1.0,  float interp = 1.0);
         //
         //      static void clearMesh(vector<Object3D *>& objects);
         //
         //      void appendPoseMesh(RigidBodyTransform pose, vector<Object3D*>& objects, Object3D *parent, int color);

         //      void generatePatchMesh(Object3D* parent, MapFrame& output,  vector<Object3D*> objects, const ApplicationState& appState);


      private:

         const int SKIP_EDGES = 5;

         std::vector<Ref<Model>> meshes;

   };

}



#endif //SRC_MESHGENERATOR_H
