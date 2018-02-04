#ifndef MP_TREE_PLANNER_HPP_
#define MP_TREE_PLANNER_HPP_

#include "MP/MPSinglePlanner.hpp"

namespace MP
{
  class MPTreePlanner : public MPSinglePlanner
  {
  public:
    MPTreePlanner(void);

    virtual ~MPTreePlanner(void);

    virtual int GetSolved(void) const
    {
      return m_vidSolved;
    }

    virtual bool IsSolved(void)
    {
      if (m_vidSolved>0)
      {
        GetPath();
        return true;
      }
      else
      {
        return false;
      }
    }

    virtual void Draw(void);
    virtual void DrawGoal(void);
    virtual void DrawCfgPath(void);
    virtual void DrawVertices();
    virtual void CompleteSetup();

    virtual void GetReversePath(const int vid, std::vector<int> * const rpath) const;
    virtual void GetPath();

    virtual double PathCost(std::vector<int> * const path) const;
    virtual MPState* GetState(const int i)
    {
      return m_vertices[i]->m_s;
    }
    virtual void GetCfgPath(void);

    double  m_dtol;
    int     m_maxNrSteps;
    int     m_minNrSteps;
    double  m_probSteer;
    double  m_goalBias;

    struct Vertex
    {
      Vertex(void)
      {
        m_parent = Constants::ID_UNDEFINED;
        m_gCost  = 0;
        m_cfg    = NULL;
        m_s      = NULL;
        m_rid    = Constants::ID_UNDEFINED;
        m_vid    = Constants::ID_UNDEFINED;
      }

      virtual ~Vertex(void)
      {
        // if(m_cfg)
        // delete m_cfg;
        // if(m_s)
        // delete[] m_s;
      }
      int       m_parent;
      int       m_vid;
      int       m_rid;
      MPState  *m_s;
      double   *m_cfg;
      double    m_gCost;
    };

    virtual int GetNrVertices(void) const
    {
      return m_vertices.size();
    }

    virtual Vertex* NewVertex(void) const
    {
      return new Vertex();
    }

    virtual void Initialize(void);

    virtual int ExtendFrom(const int vid, const double target[]);

    virtual int AddVertex(Vertex * const v);

    virtual void PrintStat(FILE * const out)
    {
      fprintf(out, "%d %f %f %f \n",
      IsSolved() > 0,
      Stats::GetSingleton()->GetValue("TotalRunTime"),
      Stats::GetSingleton()->GetValue("TimePreprocess"),
      Stats::GetSingleton()->GetValue("RobotType"));
    }

    virtual void ShowStat(void)
    {
      printf("done ... solved       = %d\n", IsSolved() > 0);
      printf("RunTime               = %f\n", Stats::GetSingleton()->GetValue("TotalRunTime"));
      printf("TimePreprocess        = %f\n", Stats::GetSingleton()->GetValue("TimePreprocess"));
      printf("RobotType             = %f\n", Stats::GetSingleton()->GetValue("RobotType"));
    }

    std::vector<Vertex*>   m_vertices;
    int                    m_vidSolved;
    double                *m_cfgTarget;

  };
}

#endif
