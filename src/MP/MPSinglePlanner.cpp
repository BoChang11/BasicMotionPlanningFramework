#include "MP/MPSinglePlanner.hpp"

namespace MP
{
  MPSinglePlanner::MPSinglePlanner(void) : MPPlanner()
  {
    if (m_robotType == RCAR)
    {
      printf("Use RobotCar\n");
      m_sim = new MPRobotCar();
    }
    else if (m_robotType == RBULL)
    {
      printf("Use RobotBullet\n");
      m_sim = new MPBVehicleSimulator();
    }
    m_sim->SetScene(&m_scene);
  }

  MPSinglePlanner::~MPSinglePlanner(void)
  {

  }

  void MPSinglePlanner::CompleteSetup()
  {
    MPPlanner::CompleteSetup();
    m_sim->SetStateFromCfg(m_abstract->m_scene->m_robotInit[m_sim->m_id]->m_cfg);
    m_sim->SetGoal(m_abstract->m_scene->m_goals[m_sim->m_id]->m_cfg);
    m_sim->CompleteSetup();
  }


}
