#include "Scenes/GenerateQuery.hpp"
#include "Utils/GDraw.hpp"

namespace MP
{

  GenerateQuery::GenerateQuery(void)
  {
    m_distToOther = 5;
    m_minDistToObs = 4;
    m_robotGoalMinDist = 15;
    m_robotGoalMaxDist = 25;
  }

  void GenerateQuery::GenerateQueries(int maxRobot,int nrQuery,const char prefix[])
  {
    for (int q = 0; q < nrQuery; q++)
    {
      printf("GenerateQuery %d\n",q );
      m_robotQuery.clear();
      m_goalsQuery.clear();

      //generate Robot
      int nrRobot = 0;
      while (nrRobot <= maxRobot)
      {
        PQPTriMesh tmesh;

        double *c1 = new double[2];
        double *c2 = new double[2];
        double *c3 = new double[2];
        double *c4 = new double[2];

        double *robotCenter = new double[2];
        double *goalCenter = new double[2];

        robotCenter[0] = RandomUniformReal(m_scene->m_grid.GetMin()[0], m_scene->m_grid.GetMax()[0]);
        robotCenter[1] = RandomUniformReal(m_scene->m_grid.GetMin()[1], m_scene->m_grid.GetMax()[1]);

        goalCenter[0] = RandomUniformReal(m_scene->m_grid.GetMin()[0], m_scene->m_grid.GetMax()[0]);
        goalCenter[1] = RandomUniformReal(m_scene->m_grid.GetMin()[1], m_scene->m_grid.GetMax()[1]);

        c1[0] = robotCenter[0]-0.5;
        c1[1] = robotCenter[1]-0.5;

        c2[0] = robotCenter[0]-0.5;
        c2[1] = robotCenter[1]+0.5;

        c3[0] = robotCenter[0]+0.5;
        c3[1] = robotCenter[1]+0.5;

        c4[0] = robotCenter[0]+0.5;
        c4[1] = robotCenter[1]-0.5;

        tmesh.Clear();
        tmesh.AddQuad(c1,c2,c3,c4);
        double distToObs = tmesh.Distance(NULL, NULL, &m_scene->m_obstacles.m_tmesh, NULL, NULL);
        if (distToObs < m_minDistToObs)
        {
          continue;
        }

        c1[0] = goalCenter[0]-0.5;
        c1[1] = goalCenter[1]-0.5;

        c2[0] = goalCenter[0]-0.5;
        c2[1] = goalCenter[1]+0.5;

        c3[0] = goalCenter[0]+0.5;
        c3[1] = goalCenter[1]+0.5;

        c4[0] = goalCenter[0]+0.5;
        c4[1] = goalCenter[1]-0.5;

        tmesh.Clear();
        tmesh.AddQuad(c1,c2,c3,c4);
        distToObs = tmesh.Distance(NULL, NULL, &m_scene->m_obstacles.m_tmesh, NULL, NULL);
        if (distToObs < m_minDistToObs)
        {
          continue;
        }

        double distRobotToGoal = Algebra2D::PointDist(robotCenter,goalCenter);
        if ((distRobotToGoal < m_robotGoalMinDist ) || (distRobotToGoal > m_robotGoalMaxDist  ) )
        {
          continue;
        }

        bool valid = true;
        for (int i = 0 ; i < (int)m_robotQuery.size(); i++)
        {
          double distRobotOtherRobot = Algebra2D::PointDist(robotCenter,m_robotQuery[i]);
          if (distRobotOtherRobot < m_distToOther )
          {
            valid = false;
            break;
          }
        }
        for (int i = 0 ; i < (int)m_goalsQuery.size(); i++)
        {
          double distRobotOtherGoal = Algebra2D::PointDist(robotCenter,m_goalsQuery[i]);
          if (distRobotOtherGoal < m_distToOther )
          {
            valid = false;
            break;
          }
        }

        for (int i = 0 ; i < (int)m_robotQuery.size(); i++)
        {
          double distGoalOtherRobot = Algebra2D::PointDist(goalCenter,m_robotQuery[i]);
          if (distGoalOtherRobot < m_distToOther )
          {
            valid = false;
            break;
          }
        }
        for (int i = 0 ; i < (int)m_goalsQuery.size(); i++)
        {
          double distGoalOtherGoal = Algebra2D::PointDist(goalCenter,m_goalsQuery[i]);
          if (distGoalOtherGoal < m_distToOther )
          {
            valid = false;
            break;
          }
        }

        if (valid == true)
        {
          m_robotQuery.push_back(robotCenter);
          m_goalsQuery.push_back(goalCenter);
          nrRobot++;
          // printf("nrRobot %d\n",nrRobot );
        }
      }

      char      cmd[300];
      sprintf(cmd, "%squery%d.txt",prefix,q);
      FILE     *queryfile = NULL;
      queryfile = fopen(cmd, "w");

      fprintf(queryfile, "Robots %d\n",(int)maxRobot);
      for (int i = 0 ; i < maxRobot; i++ )
      {
        fprintf(queryfile, "%f %f %f\n",m_robotQuery[i][0],m_robotQuery[i][1],RandomUniformReal(-M_PI,M_PI));
      }

      fprintf(queryfile, "Goals %d\n",maxRobot);
      for (int i = 0 ; i < maxRobot; i++ )
      {
        fprintf(queryfile, "%f %f 2.5\n",m_goalsQuery[i][0],m_goalsQuery[i][1]);
      }
      fclose (queryfile);
    }
  }

  void GenerateQuery::Draw(void)
  {
    for(int i = (int) m_robotQuery.size() - 1; i >= 0; --i)
    {
      const bool is2D = GDrawIs2D();
      GDrawPushTransformation();
      GDrawMultTrans(0, 0, m_scene->m_maxGroundHeight+0.3);
      GDraw2D();
      GDrawIndexColor(1);
      GDrawCircle2D(m_robotQuery[i][0], m_robotQuery[i][1],2.5);
      char      msg[100];
      sprintf(msg, "R%d", i);
      GDrawColor(1, 0.875, 0);
      GDrawString3D(msg, m_robotQuery[i][0], m_robotQuery[i][1], 0.35 , false, 2.5);
      GDrawPopTransformation();
      if(!is2D)
      GDraw3D();
    }

    for(int i = (int) m_goalsQuery.size() - 1; i >= 0; --i)
    {
      const bool is2D = GDrawIs2D();
      GDrawPushTransformation();
      GDrawMultTrans(0, 0, m_scene->m_maxGroundHeight+0.3);
      GDraw2D();
      GDrawIndexColor(0);
      GDrawCircle2D(m_goalsQuery[i][0], m_goalsQuery[i][1],2.5);
      char      msg[100];
      sprintf(msg, "G%d", i);
      GDrawColor(1, 0.875, 0);
      GDrawString3D(msg, m_goalsQuery[i][0], m_goalsQuery[i][1], 0.35 , false, 2.5);
      GDrawPopTransformation();
      if(!is2D)
      GDraw3D();
    }
  }

}
