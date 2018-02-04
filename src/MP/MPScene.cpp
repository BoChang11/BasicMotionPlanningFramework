
#include "MP/MPScene.hpp"
#include "MP/Constants.hpp"
#include "Utils/Misc.hpp"
#include "Utils/TriMeshReader.hpp"
#include "Utils/Algebra3D.hpp"
#include "Utils/GDraw.hpp"

namespace MP
{
  MPScene::MPScene(void)
  {
    m_nrRobot = -1;
    m_ground = NULL;
    m_maxGroundHeight = 0.4;
  }

  MPScene::~MPScene(void)
  {
    DeleteItems<Polygon2D*>(&(m_obstacles.m_polys));
  }

  void MPScene::ExtrudeObstacles(void)
  {
    double obstacleHeight = m_maxGroundHeight + 2;
    // m_obstacles.m_tmesh.Clear();
    double thickness = 0.5;
    m_obstacles.m_tmesh.AddBoundaries(m_grid.GetMin()[0]-thickness/2, m_grid.GetMin()[1]-thickness/2, -obstacleHeight,
    m_grid.GetMax()[0]+thickness/2, m_grid.GetMax()[1]+thickness/2, obstacleHeight,thickness);

    const int nrObs = m_obstacles.m_polys.size();
    for(int i = 0; i < nrObs; ++i)
    {
      m_obstacles.m_tmesh.AddExtrudedPolygon(m_obstacles.m_polys[i], 0.0, obstacleHeight);
    }
  }

  void MPScene::SetupFromFile(FILE * const in)
  {
    SetupScene(in);
  }

  void MPScene::SetupFromFile(FILE * const in,FILE * const query)
  {
    SetupScene(in);
    SetupQuery(query);
  }

  void MPScene::SetupScene(FILE * const in)
  {
    char keyword[100];
    char filename[100];
    FILE         *fp      = NULL;

    //read grid
    double minx, miny, maxx, maxy;
    int    dimsx, dimsy;
    if(fscanf(in, "%s %lf %lf %lf %lf %d %d",
    keyword, &minx, &miny, &maxx, &maxy, &dimsx, &dimsy) != 7)
    OnInputError(printf("Grid2 must have 6 params\n"));
    m_grid.Setup2D(dimsx, dimsy, minx, miny, maxx, maxy);

    //read view
    if(fscanf(in, "%s", keyword) != 1)
    OnInputError(printf(".. missing View data\n"));

    if(fscanf(in, "%s %lf %lf %lf",keyword, &eye[0], &eye[1], &eye[2]) != 4)
    OnInputError(printf(".. missing View data\n"));
    if(fscanf(in, "%s %lf %lf %lf",keyword, &center[0], &center[1], &center[2]) != 4)
    OnInputError(printf(".. missing View data\n"));
    if(fscanf(in, "%s %lf %lf %lf",keyword, &right[0], &right[1], &right[2]) != 4)
    OnInputError(printf(".. missing View data\n"));
    if(fscanf(in, "%s %lf %lf %lf",keyword, &forward[0], &forward[1], &forward[2]) != 4)
    OnInputError(printf(".. missing View data\n"));

    //read ground file
    if(fscanf(in, "%s %s", keyword, filename) != 2)
    OnInputError(printf(".. missing ground data\n"));
    printf("..setting up ground mesh from file <%s>\n", filename);
    if(strcmp(filename, "none") != 0)
    {
      fp = fopen(filename, "r");
      m_ground = new PQPTriMesh();
      // StandardTriMeshReader(fp, m_ground);
      TriMeshReader(filename, m_ground);
      fclose(fp);
      m_ground->AddBox(m_grid.GetMin()[0], m_grid.GetMin()[1], -0.5, m_grid.GetMax()[0], m_grid.GetMax()[1], 0);

    }
    else //if(!m_ground)
    {
      m_ground = new PQPTriMesh();
      m_ground->AddBox(m_grid.GetMin()[0], m_grid.GetMin()[1], -0.5, m_grid.GetMax()[0], m_grid.GetMax()[1], 0);
    }
    m_maxGroundHeight = m_ground->GetBoundingBoxMax()[2];

    //read obstacles file
    if(fscanf(in, "%s %s", keyword, filename) != 2)
    OnInputError(printf(".. missing obstacles data\n"));
    printf("..setting up obstacles mesh from file <%s>\n", filename);
    if(strcmp(filename, "none") != 0)
    {
      fp = fopen(filename, "r");
      m_obstacles.m_tmesh.Clear();
      TriMeshReader(filename, &m_obstacles.m_tmesh);
      fclose(fp);
    }
    else
    {
      m_obstacles.m_tmesh.Clear();
      //read custom obstacles
      int nrObs;
      if(fscanf(in, "%s %d", keyword, &nrObs) != 2)
      OnInputError(printf("..expecting Obstacles nrObs\n"));
      m_obstacles.m_polys.resize(nrObs);
      for(int i = 0; i < nrObs; ++i)
      {
        m_obstacles.m_polys[i] = new Polygon2D();
        m_obstacles.m_polys[i]->Read(in);
      }
      ExtrudeObstacles();
    }

  }

  void MPScene::SetupQuery(FILE * const in)
  {
    char keyword[100];
    char filename[100];
    FILE         *fp      = NULL;

    //read robots
    int nrRobot;
    std::vector<robotCfg*> robotInit;
    if(fscanf(in, "%s %d", keyword, &nrRobot) != 2)
    OnInputError(printf("..expecting querysize\n"));
    robotInit.resize(nrRobot);

    m_robotInit.resize(nrRobot);

    for(int i = 0; i < nrRobot; ++i)
    {
      robotInit[i] = new robotCfg();
      if(fscanf(in, "%lf %lf %lf",
      &robotInit[i]->m_cfg[0],
      &robotInit[i]->m_cfg[1],
      &robotInit[i]->m_cfg[2]) != 3)
      OnInputError(printf("..expecting TR2 for Robots %d\n", i));
    }

    for (int r = 0; r < nrRobot;r++)
    {
      m_robotInit[r] = new robotCfg();
      m_robotInit[r]->m_cfg[0] = robotInit[r]->m_cfg[0];
      m_robotInit[r]->m_cfg[1] = robotInit[r]->m_cfg[1];
      m_robotInit[r]->m_cfg[2] = robotInit[r]->m_cfg[2];
    }
    // m_nrRobot = nrRobot;

    //read goals
    int nrGoal;
    if(fscanf(in, "%s %d", keyword, &nrGoal) != 2)
    OnInputError(printf("..expecting querysize\n"));
    std::vector<Object*> goals;
    goals.resize(nrGoal);
    m_nrGoal = nrGoal;
    m_goals.resize(m_nrGoal);
    m_goalReached.resize(m_nrGoal);
    for(int i = 0; i < m_nrGoal; ++i)
    {
      m_goalReached[i] = false;
      goals[i] = new Object();
      if(fscanf(in, "%lf %lf %lf",
      &goals[i]->m_cfg[0],
      &goals[i]->m_cfg[1],
      &goals[i]->m_cfg[2]) != 3)
      OnInputError(printf("..expecting TR2 of goal %d\n", i));
    }

    for (int r = 0; r < m_nrGoal;r++)
    {
      m_goals[r] = new Object();
      m_goals[r]->m_cfg[0] = goals[r]->m_cfg[0];
      m_goals[r]->m_cfg[1] = goals[r]->m_cfg[1];
      m_goals[r]->m_cfg[2] = goals[r]->m_cfg[2];
    }

    if (m_nrRobot  == -1)
    {
      m_nrRobot = nrRobot;
    }
    m_goals.resize(m_nrRobot);
    m_robotInit.resize(m_nrRobot);
    Stats::GetSingleton()->AddValue("NrRobots", m_nrRobot);
  }

  void MPScene::GenerateQuery(int maxRobot,int nrQuery)
  {
    for (int q = 0; q < nrQuery; q++)
    {
      printf("GenerateQuery %d\n",q );

      robotQuery.clear();
      goalsQuery.clear();

      double distToOther = 5;
      double minDistToObs = 2;
      double robotGoalMinDist = 15;
      double robotGoalMaxDist = 25;

      // std::vector<double*> robotQuery;
      // std::vector<double*> goalsQuery;
      //generate Robot
      int nrRobot = 0;
      while (nrRobot<=maxRobot)
      {
        PQPTriMesh tmesh;

        double *c1 = new double[2];
        double *c2 = new double[2];
        double *c3 = new double[2];
        double *c4 = new double[2];

        double *robotCenter = new double[2];
        double *goalCenter = new double[2];

        robotCenter[0] = RandomUniformReal(m_grid.GetMin()[0], m_grid.GetMax()[0]);
        robotCenter[1] = RandomUniformReal(m_grid.GetMin()[1], m_grid.GetMax()[1]);

        goalCenter[0] = RandomUniformReal(m_grid.GetMin()[0], m_grid.GetMax()[0]);
        goalCenter[1] = RandomUniformReal(m_grid.GetMin()[1], m_grid.GetMax()[1]);

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
        double distToObs = tmesh.Distance(NULL, NULL, &m_obstacles.m_tmesh, NULL, NULL);
        if (distToObs < minDistToObs)
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
        distToObs = tmesh.Distance(NULL, NULL, &m_obstacles.m_tmesh, NULL, NULL);
        if (distToObs < minDistToObs)
        {
          continue;
        }

        double distRobotToGoal = Algebra2D::PointDist(robotCenter,goalCenter);
        if ((distRobotToGoal < robotGoalMinDist ) || (distRobotToGoal > robotGoalMaxDist ) )
        {
          continue;
        }

        bool valid = true;
        for (int i = 0 ; i < (int)robotQuery.size(); i++)
        {
          double distRobotOtherRobot = Algebra2D::PointDist(robotCenter,robotQuery[i]);
          if (distRobotOtherRobot < distToOther )
          {
            valid = false;
            break;
          }
        }
        for (int i = 0 ; i < (int)goalsQuery.size(); i++)
        {
          double distRobotOtherGoal = Algebra2D::PointDist(robotCenter,goalsQuery[i]);
          if (distRobotOtherGoal < distToOther )
          {
            valid = false;
            break;
          }
        }

        for (int i = 0 ; i < (int)robotQuery.size(); i++)
        {
          double distGoalOtherRobot = Algebra2D::PointDist(goalCenter,robotQuery[i]);
          if (distGoalOtherRobot < distToOther )
          {
            valid = false;
            break;
          }
        }
        for (int i = 0 ; i < (int)goalsQuery.size(); i++)
        {
          double distGoalOtherGoal = Algebra2D::PointDist(goalCenter,goalsQuery[i]);
          if (distGoalOtherGoal < distToOther )
          {
            valid = false;
            break;
          }
        }

        if (valid == true)
        {
          robotQuery.push_back(robotCenter);
          goalsQuery.push_back(goalCenter);
          nrRobot++;
          // printf("nrRobot %d\n",nrRobot );
        }
      }
      char      cmd[300];
      FILE     *queryfile = NULL;
      sprintf(cmd, "sceneEquery%d.txt",q);
      queryfile = fopen(cmd, "w");
      fprintf(queryfile, "Robots %d\n",(int)maxRobot);
      for (int i = 0 ; i < maxRobot; i++ )
      {
        fprintf(queryfile, "%f %f %f\n",robotQuery[i][0],robotQuery[i][1],RandomUniformReal(-M_PI,M_PI));
      }

      fprintf(queryfile, "Goals %d\n",maxRobot);
      for (int i = 0 ; i < maxRobot; i++ )
      {
        fprintf(queryfile, "%f %f 2.5\n",goalsQuery[i][0],goalsQuery[i][1]);
      }
      fclose (queryfile);
    }


  }

  void MPScene::Draw(void)
  {
    DrawGround();
    DrawObstacles();
    DrawGoals();
  }

  void MPScene::DrawQuery(void)
  {
    for(int i = (int) robotQuery.size() - 1; i >= 0; --i)
    {
      const bool is2D = GDrawIs2D();
      GDrawPushTransformation();
      GDrawMultTrans(0, 0, m_maxGroundHeight+0.3);
      GDraw2D();
      GDrawIndexColor(1);
      GDrawCircle2D(robotQuery[i][0], robotQuery[i][1],2.5);
      char      msg[100];
      sprintf(msg, "R%d", i);
      GDrawColor(1, 0.875, 0);
      GDrawString3D(msg, robotQuery[i][0], robotQuery[i][1], 0.35 , false, 2.5);
      GDrawPopTransformation();
      if(!is2D)
      GDraw3D();
    }

    for(int i = (int) goalsQuery.size() - 1; i >= 0; --i)
    {
      const bool is2D = GDrawIs2D();
      GDrawPushTransformation();
      GDrawMultTrans(0, 0, m_maxGroundHeight+0.3);
      GDraw2D();
      GDrawIndexColor(0);
      GDrawCircle2D(goalsQuery[i][0], goalsQuery[i][1],2.5);
      char      msg[100];
      sprintf(msg, "G%d", i);
      GDrawColor(1, 0.875, 0);
      GDrawString3D(msg, goalsQuery[i][0], goalsQuery[i][1], 0.35 , false, 2.5);
      GDrawPopTransformation();
      if(!is2D)
      GDraw3D();
    }
  }


  void MPScene::DrawGoals(void)
  {
    for(int i = (int) m_goals.size() - 1; i >= 0; --i)
    {
      const bool is2D = GDrawIs2D();
      GDrawPushTransformation();
      GDrawMultTrans(0, 0, m_maxGroundHeight+0.3);
      GDraw2D();
      if (m_goalReached[i] == false)
      {
        GDrawIndexColor(0);
      }
      else
      {
        GDrawIndexColor(1);
      }
      GDrawCircle2D(m_goals[i]->m_cfg[0], m_goals[i]->m_cfg[1],2.5);
      char      msg[100];
      sprintf(msg, "G%d", i);
      GDrawColor(1, 0.875, 0);
      GDrawString3D(msg, m_goals[i]->m_cfg[0], m_goals[i]->m_cfg[1], 0.35 , false, 2.5);
      GDrawPopTransformation();
      if(!is2D)
      GDraw3D();
    }
  }

  void MPScene::DrawScene(void)
  {
    glEnable(GL_LIGHTING);
    GDraw2D();
    GDrawPushTransformation();
    GDrawColor(0.1, 0.51, 0.561);
    GDrawMultTrans(0, 0, Constants::SCENE_DRAWZ_ROOM_BOX);
    GDrawAABox2D(m_grid.GetMin(), m_grid.GetMax());
    GDrawPopTransformation();
    glDisable(GL_LIGHTING);
  }

  void MPScene::DrawGround(void)
  {
    glEnable(GL_LIGHTING);
    GDrawPushTransformation();
    GDraw3D();
    if (m_ground== NULL)
    return;
    GMaterial gmat;
    gmat.SetRuby();
    gmat.SetDiffuse(0.09,0.2 , 0.27);
    GDrawMaterial(&gmat);
    m_ground->DrawWithEdges();
    GDrawPopTransformation();
    glDisable(GL_LIGHTING);
  }

  void MPScene::DrawObstacles(void)
  {
    glEnable(GL_LIGHTING);
    GDrawPushTransformation();

    GDraw3D();
    GMaterial gmat;
    gmat.SetRuby();
    gmat.SetDiffuse(0.455, 0.51, 0.561);
    GDrawMaterial(&gmat);
    m_obstacles.m_tmesh.Draw();
    GDrawPopTransformation();

    GDrawPushTransformation();
    GDraw2D();
    GDrawColor(0.455, 0.51, 0.561);
    GDrawMultTrans(0, 0, Constants::SCENE_DRAWZ_ROOM_BOX);
    GDrawPopTransformation();
    glDisable(GL_LIGHTING);

  }

}
