#ifndef MP_RRT_HPP_
#define MP_RRT_HPP_

#include "MP/MPProximityPlanner.hpp"

namespace MP
{
  class MPRRTPlanner : public MPProximityPlanner
  {
  public:
    MPRRTPlanner(void) : MPProximityPlanner()
    {
    }

    virtual ~MPRRTPlanner(void)
    {
    }

    virtual void Run(const int nrIters);
  };
}

#endif
