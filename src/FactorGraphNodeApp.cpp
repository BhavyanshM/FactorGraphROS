//
// Created by quantum on 9/17/21.
//

#include "Core/Application.h"
#include "Core/Clay.h"
#include "FactorGraphNodeLayer.h"

class FactorGraphNodeApp : public Clay::Application
{
   public:
      FactorGraphNodeApp(int argc, char** argv);
      ~FactorGraphNodeApp() = default;
};

FactorGraphNodeApp::FactorGraphNodeApp(int argc, char** argv)
{
   //   PushLayer(new ExampleLayer());
   Clay::FactorGraphNodeLayer* app = new Clay::FactorGraphNodeLayer(argc, argv);
   PushLayer(app);
}

int main(int argc, char** argv)
{
   Clay::Log::Init();   CLAY_LOG_INFO("Welcome to Clay Engine!");

   CLAY_PROFILE_BEGIN_SESSION("Startup", "ClayProfile-Startup.json");
   FactorGraphNodeApp app(argc, argv);
   CLAY_PROFILE_END_SESSION();

   CLAY_PROFILE_BEGIN_SESSION("Runtime", "ClayProfile-Runtime.json");
   app.Run();
   CLAY_PROFILE_END_SESSION();

   return 0;
}