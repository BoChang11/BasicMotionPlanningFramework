#include "Utils/GManager.hpp"
#include "Utils/GDraw.hpp"
#include "Utils/Timer.hpp"
#include "Utils/Algebra3D.hpp"
#include "Utils/PseudoRandom.hpp"
#include "Utils/Flags.hpp"
#include "Utils/Misc.hpp"
#include "MP/MPRobotCar.hpp"
#include "MP/MPBVehicleSimulator.hpp"
#include "MP/MPScene.hpp"
#include "Utils/TriMeshReader.hpp"

namespace MP
{
  class GRunSimulator : public GManager
  {
  public:
    enum
    {
      FLAG_STEER = 1,
      FLAG_MOVE = 2
    };

    GRunSimulator(void) : GManager()
    {
      m_timer   = 30;
      m_flags   = 0;
      m_target[0] = m_target[1] = m_target[2] = 0;

      GlobalCamera();
    }

    virtual ~GRunSimulator(void)
    {
    }

    virtual void CompleteSetup()
    {

      printf("heherherherhher\n" );
      //setup scene
      m_scene.m_grid.Setup2D(128, 128, -40, -40, 40, 40);
      m_scene.eye[0] = 0.000000;
      m_scene.eye[1] = -131.433051;
      m_scene.eye[2] = 168.708779;
      m_scene.center[0] = 0.000000;
      m_scene.center[1] = 0.000000;
      m_scene.center[2] = 0.000000;
      m_scene.right[0] = 1.000000;
      m_scene.right[1] =  0.000000;
      m_scene.right[2 ]= 0.000000;
      m_scene.forward[0] = 0.000000;
      m_scene.forward[1] = 0.601815;
      m_scene.forward[2] = -0.798636;

      m_scene.CompleteSetup();

      if (m_simType == 0)
      {
        m_scene.m_ground = new PQPTriMesh();
        m_scene.m_ground->AddBox(m_scene.m_grid.GetMin()[0], m_scene.m_grid.GetMin()[1], -0.5, m_scene.m_grid.GetMax()[0], m_scene.m_grid.GetMax()[1], 0);
      }
      else
      {
        const char * groundMeshFile="textures/ground.tmesh";
        m_scene.m_ground = new PQPTriMesh();
        // StandardTriMeshReader(fp, m_ground);
        TriMeshReader(groundMeshFile, m_scene.m_ground);
        m_scene.m_ground->AddBox(m_scene.m_grid.GetMin()[0], m_scene.m_grid.GetMin()[1], -0.5, m_scene.m_grid.GetMax()[0],m_scene.m_grid.GetMax()[1], 0);
      }

      if (m_simType == 0)
      {
        m_sim = new MPRobotCar();
      }
      else
      {
        m_sim = new MPBVehicleSimulator();
      }
      m_sim->m_id = 0;
      m_sim->SetScene(&m_scene);
      double init[3];
      init[0] = 0.0;
      init[1] = 0.0;
      init[2] = 0.0;

      m_sim->SetStateFromCfg(init);
      m_sim->CompleteSetup();


      m_gCamera.SetEyeCenterRightForward(m_scene.eye,
        m_scene.center,
        m_scene.right,
        m_scene.forward);

    }

    virtual void HandleEventOnDisplay(void)
    {
      GManager::HandleEventOnDisplay();
      m_scene.Draw();
      m_sim->Draw();

      if(HasFlag(m_flags, FLAG_STEER))
      {
        GDrawColor(1.0, 0.0, 0.0);
        GDrawSphere3D(m_target, 1.0);
      }

    }

    virtual bool HandleEventOnNormalKeyPress(const int key)
    {
      if(key == 's')
      {
        m_flags = FlipFlag(m_flags, FLAG_STEER);
      }
      return GManager::HandleEventOnNormalKeyPress(key);
    }

    virtual bool HandleEventOnMouseLeftBtnDown(const int x, const int y)
    {
      MousePosFromScreenToWorld(x, y, &m_target[0], &m_target[1], &m_target[2]);
      if(HasFlag(m_flags, FLAG_STEER))
      {
        m_sim->StartSteerToPosition();
      }
      return true;
    }

    virtual void HandleEventOnTimer(void)
    {
      if(HasFlag(m_flags, FLAG_STEER))
      {
        m_sim->SteerToPosition(m_target);
        m_sim->SimulateOneStep();
      }

      GManager::HandleEventOnTimer();
    }

    void GlobalCamera(void)
    {
      const double eye[] = {0.000000, -129.326207, 117.180340};
	    const double center[] = {0.000000, 0.000000, 0.000000};
	    const double right[] = {1.000000, -0.000000, 0.000000};
	    const double forward[] = {0.000000, 0.731354, -0.681998};
      m_gCamera.SetEyeCenterRightForward(eye, center, right, forward);
    }

    Flags     m_flags;
    double    m_target[3];
    MPScene   m_scene;
    MPSimulator  *m_sim;
    int           m_simType;
  };
};


extern "C" int GRunSimulator(int argc, char **argv)
{
  MP::GRunSimulator gManager;
  gManager.m_simType      = argc > 1 ? atoi(argv[1]) : 0;
  gManager.CompleteSetup();
  gManager.MainLoop("GRunProblem", 1280, 720);

  return 0;
}
