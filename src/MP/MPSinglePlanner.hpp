#ifndef MP_SINGLE_PLANNER_HPP_
#define MP_SINGLE_PLANNER_HPP_

#include "MP/MPPlanner.hpp"

namespace MP
{
  class MPSinglePlanner : public MPPlanner
  {
  public:
    MPSinglePlanner(void);
    virtual ~MPSinglePlanner(void);
    virtual void CompleteSetup();

    MPSimulator           *m_sim;
    std::vector<double*>   m_cfgPath;
    std::vector<MPState*>  m_statePath;
    std::vector<int>       m_path;

  };
}

#endif
