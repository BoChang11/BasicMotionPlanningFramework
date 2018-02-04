#include "MP/MPSimulator.hpp"
#include "Utils/Constants.hpp"

namespace MP
{
  MPSimulator::MPSimulator(void)
  {
    m_dt             = Constants::SIMULATOR_TIME_STEP;
    m_minDistOneStep = Constants::SIMULATOR_MIN_DISTANCE_ONE_STEP;
    m_maxDistOneStep = Constants::SIMULATOR_MAX_DISTANCE_ONE_STEP;
    m_scene          = NULL;
    m_id             = 0;
    m_length         = Constants::ROBOT_LENGTH;
    m_width          = Constants::ROBOT_WIDTH;
    m_height         = Constants::ROBOT_HEIGHT;
    m_cfgUseRot      = false;

  }

  MPSimulator::~MPSimulator(void)
  {
  }

  void MPSimulator::DrawCfgShape(const double cfg[])
  {
    double R[Algebra3D::Rot_NR_ENTRIES];
    double T3[] = {cfg[0], cfg[1], m_scene->m_maxGroundHeight+0.3};
    Algebra3D::ZAxisAngleAsRot(cfg[2], R);
    GDrawPushTransformation();
    GDraw2D();
    GDrawColor(1.0, 0.8, 0.8);
    GDrawMultTransRot(T3, R);
    GMaterial *gmat;
    gmat = new GMaterial();
    gmat->SetDiffuse(0.7, 0.03, 0.03);
    gmat->SetAmbient(0.0, 0.0, 0.0, 1.0);
    gmat->SetSpecular(0.0, 0.0, 0.0, 1.0);
    gmat->SetEmmissive(0.0, 0.0, 0.0, 1.0);
    gmat->SetShininess(0);
    m_projShape.SetMainMaterial(gmat);
    glDisable(GL_LIGHTING);
    m_projShape.Draw();
    GDrawPopTransformation();
    glEnable(GL_LIGHTING);
  }

  bool MPSimulator::IsCfgValid(double cfg[])
  {
    return MinDistFromCfgToObstacles(cfg) > m_bodyLength/4;
  }

  bool MPSimulator::IsCfgValid(void)
  {
    minDistToObs = MinDistFromCfgToObstacles(GetCfg());
    return minDistToObs > 0.1;
  }


}
