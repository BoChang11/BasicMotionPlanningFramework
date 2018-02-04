#ifndef MP_PLANNER_HPP_
#define MP_PLANNER_HPP_

#include "Utils/GDraw.hpp"
#include "MP/MPSimulator.hpp"
#include "MP/MPRobotCar.hpp"
#include "MP/MPBVehicleSimulator.hpp"
#include "MP/MPAbstraction.hpp"
#include "MP/MPScene.hpp"
#include "MP/MPTriAbstraction.hpp"
#include "MP/MPPrmAbstraction.hpp"

namespace MP
{
  class MPPlanner
  {
  public:
    enum Type
    {
      PRM     = 0,
      TRI     = 1,
      RCAR    = 0,
      RBULL   = 1
    };
    MPPlanner(void);
    virtual ~MPPlanner(void)
    {
    }
    virtual void CompleteSetup();
    virtual void Run(const int nrIters) = 0;
    virtual void Draw(void) = 0;

    virtual void SetAbstraction(MPAbstraction *abstract)
    {
      m_abstract = abstract;
    }

    virtual bool IsSolved(void) = 0;
    virtual void PrintStat(FILE * const out) = 0;
    virtual void ShowStat(void) = 0;

    MPAbstraction       *m_abstract;
    MPScene              m_scene;
    int                  m_abstractionType;
    int                  m_robotType;

  };
}

#endif
