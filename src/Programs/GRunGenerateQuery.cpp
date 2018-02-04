#include "Utils/GManager.hpp"
#include "Utils/GDraw.hpp"
#include "Utils/Timer.hpp"
#include "Utils/Algebra3D.hpp"
#include "Utils/PseudoRandom.hpp"
#include "Utils/Flags.hpp"
#include "Utils/Misc.hpp"
#include "MP/MPScene.hpp"
#include "MP/MPAbstraction.hpp"
#include "MP/MPPrmAbstraction.hpp"
#include "MP/MPTriAbstraction.hpp"
#include "MP/MPTriAbstraction.hpp"
#include "Scenes/GenerateQuery.hpp"

namespace MP
{
  class GRunGenerateQuery : public GManager
  {
  public:

    GRunGenerateQuery(void) : GManager()
    {
      m_timer   = 50;
      m_flags   = 0;
      m_target[0] = m_target[1] = m_target[2] = 0;
      GlobalCamera();
    }

    virtual ~GRunGenerateQuery(void)
    {
    }

    virtual void CompleteSetup()
    {
      m_queryMaker.SetScene(&m_scene);

      m_gCamera.SetEyeCenterRightForward(m_scene.eye,
        m_scene.center,
        m_scene.right,
        m_scene.forward);
    }

    virtual void HandleEventOnDisplay(void)
    {
      GManager::HandleEventOnDisplay();
      m_scene.Draw();
      m_queryMaker.Draw();

    }

    virtual bool HandleEventOnNormalKeyPress(const int key)
    {
      if(key == 'g')
      {
        m_queryMaker.GenerateQueries(20,1,m_prefix);
      }

      return GManager::HandleEventOnNormalKeyPress(key);
    }

    virtual bool HandleEventOnMouseLeftBtnDown(const int x, const int y)
    {
      MousePosFromScreenToWorld(x, y, &m_target[0], &m_target[1], &m_target[2]);
      printf("%f %f 0\n",m_target[0],m_target[1]);
      return true;
    }

    virtual void HandleEventOnTimer(void)
    {
      GManager::HandleEventOnTimer();
    }

    void GlobalCamera(void)
    {
      const double eye[] = {0.000000, -234.708158, 209.209480};
      const double center[] = {0.000000, 0.000000, 0.000000};
      const double right[] = {1.000000, -0.000000, 0.000000};
      const double forward[] = {0.000000, 0.731354, -0.681998};
      m_gCamera.SetEyeCenterRightForward(eye, center, right, forward);
    }

    MPScene         m_scene;
    double          m_target[3];
    Flags           m_flags;
    GenerateQuery   m_queryMaker;
    char            m_prefix[300];
  };
};


extern "C" int GRunGenerateQuery(int argc, char **argv)
{
  MP::GRunGenerateQuery gManager;

  FILE *in            = argc > 1 ? fopen(argv[1], "r") : NULL;
  const char  *prefix = argc > 2 ? argv[2] : "";

  if (argc > 1)
  {
    sprintf(gManager.m_prefix, "%s",prefix);
    gManager.m_scene.SetupFromFile(in);
    gManager.m_scene.CompleteSetup();
    gManager.CompleteSetup();
    fclose(in);
  }

  gManager.MainLoop("GRunProblem", 1280, 720);

  return 0;
}
