#include "Utils/GManager.hpp"
#include "Utils/GDraw.hpp"
#include "Utils/Timer.hpp"
#include "Utils/Algebra3D.hpp"
#include "Utils/PseudoRandom.hpp"
#include "Utils/Flags.hpp"
#include "MP/MPGUSTPlanner.hpp"
#include "MP/MPRRTPlanner.hpp"
#include "MP/MPPlanner.hpp"
#include "MP/MPSinglePlanner.hpp"

namespace MP
{
  class GRunPlanner : public GManager
  {
  public:
    enum
    {
      FLAG_DRAW_TREE = 1,
      FLAG_PAUSE = 2,
      FLAG_STEER = 4,
      FLAG_RUN  = 8,
      RRTPlanner =0,
      EPPlanner = 1
    };

    GRunPlanner(void) : GManager()
    {
      m_timer   = 50;
      m_flags   = FLAG_DRAW_TREE;
      GlobalCamera();
    }

    virtual ~GRunPlanner(void)
    {
    }

    virtual void CompleteSetup()
    {

      m_planner->m_scene.CompleteSetup();
      m_planner->CompleteSetup();

      m_gCamera.SetEyeCenterRightForward(m_planner->m_scene.eye,
        m_planner->m_scene.center,
        m_planner->m_scene.right,
        m_planner->m_scene.forward);
    }

      virtual void HandleEventOnDisplay(void)
      {
      GManager::HandleEventOnDisplay();
      m_planner->Draw();
    }

    virtual bool HandleEventOnNormalKeyPress(const int key)
    {
      if(key == 'r')
      {
        if (m_planner->IsSolved() == false)
        {
          m_planner->Run(100);
          m_pathPos = 0;
        }
        else
        {
          m_pathPos = 0;
        }
      }

      return GManager::HandleEventOnNormalKeyPress(key);
    }

    virtual bool HandleEventOnMouseLeftBtnDown(const int x, const int y)
    {
      double m_target[3];
      MousePosFromScreenToWorld(x, y, &m_target[0], &m_target[1], &m_target[2]);
      printf("click xy: %f %f \n",m_target[0],m_target[1]);
      return true;
    }

    virtual void HandleEventOnTimer(void)
    {
      if(HasFlag(m_flags, FLAG_PAUSE))
      return;

      if (m_planner->IsSolved() == true)
      {
        m_planner->m_sim->SetState(m_planner->m_statePath[m_pathPos]);
        m_pathPos++;
        if (m_pathPos >= m_planner->m_statePath.size())
        {
          m_pathPos = m_planner->m_statePath.size()-1;
        }
      }

      GManager::HandleEventOnTimer();
    }

    void GlobalCamera(void)
    {
      const double eye[] = {0.000000, -294.782392, 278.219162};
      const double center[] = {0.000000, 0.000000, 0.000000};
      const double right[] = {1.000000, -0.000000, 0.000000};
      const double forward[] = {0.000000, 0.731354, -0.681998};
      m_gCamera.SetEyeCenterRightForward(eye, center, right, forward);

    }

    Flags            m_flags;
    std::vector<int> m_path;
    int              m_pathPos;
    MPSinglePlanner *m_planner;
  };
};


extern "C" int GRunPlanner(int argc, char **argv)
{
  enum
  {
    RRTPlanner =0,
    GUSTPlanner = 1
  };

  MP::GRunPlanner gManager;

  FILE *in         = argc > 1 ? fopen(argv[1], "r") : NULL;
  FILE *query      = argc > 2 ? fopen(argv[2], "r") : NULL;
  int plannerType  = argc > 3 ? atoi(argv[3]) : 1;

  if(in&&query)
  {
    if (plannerType == RRTPlanner)
    {
      gManager.m_planner = new MP::MPRRTPlanner();
    }
    if (plannerType == EPPlanner)
    {
      gManager.m_planner = new MP::MPGUSTPlanner();
    }
    gManager.m_planner->m_scene.SetupFromFile(in,query);
    gManager.CompleteSetup();

    fclose(in);
  }
  gManager.MainLoop("GRunPlanner", 1280, 720);

  return 0;
}
