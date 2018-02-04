#ifndef MP_CREATE_SCENE_HPP_
#define MP_CREATE_SCENE_HPP_

#include "MP/MPScene.hpp"

namespace MP
{
  class GenerateQuery
  {
  public:
    GenerateQuery(void);
    virtual ~GenerateQuery(void)
    {

    }

    virtual void SetScene(MPScene *scene)
    {
      m_scene = scene;
    }

    virtual void Draw(void);
    virtual void GenerateQueries(int maxRobot,int nrQuery,const char prefix[]);

    MPScene          *m_scene;

    std::vector<double*> m_robotQuery;
    std::vector<double*> m_goalsQuery;

    double m_distToOther;
    double m_minDistToObs;
    double m_robotGoalMinDist;
    double m_robotGoalMaxDist;
  };
}

#endif
