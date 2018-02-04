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

namespace MP
{
  class GRunScene : public GManager
  {
  public:

    GRunScene(void) : GManager()
    {
      m_timer   = 50;
      m_flags   = 0;
      m_target[0] = m_target[1] = m_target[2] = 0;
      GlobalCamera();
    }

    virtual ~GRunScene(void)
    {
    }

    virtual void CompleteSetup()
    {
      m_gCamera.SetEyeCenterRightForward(m_scene.eye,
        m_scene.center,
        m_scene.right,
        m_scene.forward);

      if (m_hasAbstract == true)
      {
        //gManager.m_abstract = new MPTriAbstraction();
        m_abstract = new MPPrmAbstraction();
        m_abstract->SetScene(&m_scene);
        m_abstract->CompleteSetup();
      }
    }

    virtual void HandleEventOnDisplay(void)
    {
      GManager::HandleEventOnDisplay();
      m_scene.Draw();
      if (m_hasAbstract == true)
      {
        m_abstract->Draw();
      }
    }

    virtual bool HandleEventOnNormalKeyPress(const int key)
    {
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
    MPAbstraction  *m_abstract;
    double          m_target[3];
    Flags           m_flags;
    bool            m_hasAbstract;
  };
};


extern "C" int GRunScene(int argc, char **argv)
{
  MP::GRunScene gManager;

  FILE *in = argc > 1 ? fopen(argv[1], "r") : NULL;
  FILE *query = argc > 2 ? fopen(argv[2], "r") : NULL;
  gManager.m_hasAbstract = false;

  if(argc > 2)
  {
    gManager.m_scene.SetupFromFile(in,query);
    gManager.m_scene.CompleteSetup();
    gManager.m_hasAbstract = true;
    gManager.CompleteSetup();
    fclose(in);
    fclose(query);
  }
  else if (argc > 1)
  {
    gManager.m_scene.SetupFromFile(in);
    gManager.m_scene.CompleteSetup();
    gManager.CompleteSetup();
    fclose(in);
  }
  else
  {
    gManager.m_scene.CompleteSetup();
    gManager.CompleteSetup();
  }


  gManager.MainLoop("GRunProblem", 1280, 720);

  return 0;
}
