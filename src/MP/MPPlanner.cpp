#include "MP/MPPlanner.hpp"

namespace MP
{
  MPPlanner::MPPlanner(void)
  {
    m_abstractionType = PRM;
    m_robotType = RCAR;
  }

  void MPPlanner::CompleteSetup()
  {
    m_scene.CompleteSetup();

    if (m_abstractionType == TRI)
    {
      m_abstract = new MPTriAbstraction();
    }
    else if (m_abstractionType == PRM)
    {
      m_abstract = new MPPrmAbstraction();
    }
    m_abstract->SetScene(&m_scene);

    Timer::Clock clk;
    Timer::Start(&clk);
    m_abstract->CompleteSetup();
    Stats::GetSingleton()->AddValue("TimePreprocess", Timer::Elapsed(&clk));
  }
}
