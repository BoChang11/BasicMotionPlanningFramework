#include "MP/MPTreePlanner.hpp"
#include "Utils/Colormap.hpp"
#include <algorithm>

namespace MP
{
  MPTreePlanner::MPTreePlanner(void) : MPSinglePlanner()
  {
    m_dtol            = 5;
    m_maxNrSteps      = 100;
    m_minNrSteps      = 20;
    m_vidSolved       = Constants::ID_UNDEFINED;
    m_probSteer       = 1;
    m_goalBias        = 0.05;
    m_cfgTarget       = NULL;
  }

  MPTreePlanner::~MPTreePlanner(void)
  {
    DeleteItems<Vertex*>(&m_vertices);
    if(m_cfgTarget)
    delete[] m_cfgTarget;
  }


  void MPTreePlanner::CompleteSetup()
  {
    MPSinglePlanner::CompleteSetup();
    m_sim->SetStateFromCfg(m_abstract->m_scene->m_robotInit[m_sim->m_id]->m_cfg);
    m_sim->SetGoal(m_abstract->m_scene->m_goals[m_sim->m_id]->m_cfg);
    m_sim->CompleteSetup();
  }

  void MPTreePlanner::Initialize(void)
  {
    if(m_cfgTarget == NULL)
    m_cfgTarget = m_sim->NewCfg();

    if(!(m_sim->IsStateValid()))
    {
      printf("initial state is in collision\n");
      exit(0);
    }

    Vertex *v   = NewVertex();
    if(AddVertex(v) < 0)
    {
      printf("initial state invalid\n");
      delete v;
      exit(0);
    }
  }

  int MPTreePlanner::AddVertex(Vertex * const v)
  {
    if(v->m_cfg == NULL)
    {
      v->m_cfg = m_sim->NewCfg();
    }
    m_sim->GetCfg(v->m_cfg);
    v->m_s = m_sim->NewState();
  	m_sim->GetState(v->m_s);

    m_vertices.push_back(v);
    const int vid = m_vertices.size() - 1;

    if(m_sim->IsGoalReached(v->m_cfg))
    {
      m_vidSolved = vid;
      //GetCfgPath();
    }

    return vid;
  }

  void MPTreePlanner::GetReversePath(const int vid, std::vector<int> * const rpath) const
  {
    rpath->clear();

    int pid = vid;
    while(pid >= 0)
    {
      rpath->push_back(pid);
      if(pid >= (int) m_vertices.size())
      printf("path is wrong ... pid = %d nv = %d size=%d\n", pid, (int)m_vertices.size(), (int)rpath->size());
      pid = m_vertices[pid]->m_parent;
    }
    ReverseItems<int>(rpath);
  }

  void MPTreePlanner::GetPath()
  {
    m_statePath.clear();
    m_cfgPath.clear();
    m_path.clear();

    int pid = m_vidSolved;
    while(pid >= 0)
    {
      m_path.push_back(pid);
      if(pid >= (int) m_vertices.size())
      printf("path is wrong ... pid = %d nv = %d size=%d\n ", pid, (int) m_vertices.size(), (int) m_path.size());
      pid = m_vertices[pid]->m_parent;
    }
    std::reverse(m_path.begin(),m_path.end());
    for (int i = 0; i<(int)m_path.size();++i)
    {
      m_statePath.push_back(m_vertices[m_path[i]]->m_s);
      m_cfgPath.push_back(m_vertices[m_path[i]]->m_cfg);
    }
  }

  void MPTreePlanner::GetCfgPath(void)
  {
    std::vector<int> path;
    path.clear();
    int pid = m_vidSolved;
    while(pid >= 0)
    {
      path.push_back(pid);

      if(pid >= (int) m_vertices.size())
      printf("path is wrong ... pid = %d nv = %d size=%d\n ", pid, (int) m_vertices.size(), (int) path.size());

      pid = m_vertices[pid]->m_parent;
    }
    std::reverse(path.begin(),path.end());    // 9 8 7 6 5 4 3 2 1

    m_cfgPath.clear();
    for (int i = 0; i<path.size();++i)
    {
      m_cfgPath.push_back(m_vertices[path[i]]->m_cfg);
    }
  }

  double MPTreePlanner::PathCost(std::vector<int> * const path) const
  {
    double c = 0;
    for(int i = 1; i < path->size(); ++i)
    c += m_sim->DistanceCfg(m_vertices[(*path)[i-1]]->m_cfg,
    m_vertices[(*path)[i]]->m_cfg);
    return c;
  }

  int MPTreePlanner::ExtendFrom(const int vid,const double target[])
  {
    int        parent  = vid;
    int        i       = 0;
    const bool steer   = RandomUniformReal() < m_probSteer;
    const int  nrSteps = RandomUniformInteger(m_minNrSteps, m_maxNrSteps);
    int        count   = 0;

    m_sim->SetState(m_vertices[vid]->m_s);

    if(steer)
    m_sim->StartSteerToPosition();
    else
    m_sim->SampleControl();

    for(i = 0; i < nrSteps && GetSolved() < 0; ++i)
    {
      if(steer)
      m_sim->SteerToPosition(target);

      m_sim->SimulateOneStep();
      if(!m_sim->IsStateValid())
      return i;

      Vertex *vnew = NewVertex();
      vnew->m_parent = parent;
      if(AddVertex(vnew) < 0)
      {
        delete vnew;
        return i;
      }

      if(steer && m_sim->HasReachedPosition(target, m_dtol))
      return i;

      parent = m_vertices.size() - 1;
    }

    return i;
  }

  void MPTreePlanner::Draw(void)
  {
    m_scene.Draw();
    m_sim->Draw();
    if (IsSolved() == false)
    {
      // m_abstract->DrawRegions();
      // m_abstract->DrawEdges();
    }
    DrawVertices();
    DrawGoal();
  }

  void MPTreePlanner::DrawVertices()
  {
    const bool is2D = GDrawIs2D();
    GDrawPushTransformation();
    GDrawLineWidth(4.0);
    GDrawIndexColor(0);
    GDrawMultTrans(0, 0, m_scene.m_maxGroundHeight+0.4);

    for(int i = m_vertices.size() - 1; i >= 1; --i)
    {
      GDrawSegment2D(m_vertices[i]->m_cfg, m_vertices[m_vertices[i]->m_parent]->m_cfg);
    }

    GDrawPopTransformation();
    if(!is2D)
    GDraw3D();
  }

  void MPTreePlanner::DrawGoal(void)
  {
    GDrawPushTransformation();
    char      msg[100];
    GMaterial gmat;
    GDrawMaterial(&gmat);
    sprintf(msg, "G%d", m_sim->m_id);
    gmat.SetObsidian();
    GDrawMaterial(&gmat);
    GDrawString3D(msg, m_sim->m_goal[0], m_sim->m_goal[1], m_scene.m_maxGroundHeight+ 0.04 , false, 2.0);

    GDrawPopTransformation();
  }

  void MPTreePlanner::DrawCfgPath(void)
  {
    const bool is2D = GDrawIs2D();

    GDrawPushTransformation();
    GDrawMultTrans(0, 0, 0.3);
    GDraw2D();
    GDrawIndexColor(m_sim->m_id);

    // printf("m_cfgPathsize %d robotid %d\n",m_cfgPath.size(),m_id );
    GDrawLineWidth(3);
    for(int i = m_cfgPath.size() - 1; i >= 0; --i)
    {
      GDrawCircle2D(m_cfgPath[i][0],m_cfgPath[i][1], 0.3);
    }
    GDrawPopTransformation();

    if(!is2D)
    GDraw3D();
  }

}
