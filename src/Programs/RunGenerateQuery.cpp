#include "Utils/GManager.hpp"
#include "Utils/GDraw.hpp"
#include "Utils/Timer.hpp"
#include "Utils/Algebra3D.hpp"
#include "Utils/PseudoRandom.hpp"
#include "Utils/Flags.hpp"
#include "Utils/Misc.hpp"
#include "MP/MPScene.hpp"
#include "MP/MPAbstraction.hpp"
#include "MP/MPPrmAbstraction.hpp"
#include "MP/MPTriAbstraction.hpp"
#include "MP/MPTriAbstraction.hpp"
#include "Scenes/GenerateQuery.hpp"

using namespace MP;
extern "C" int RunGenerateQuery(int argc, char **argv)
{

  GenerateQuery   m_queryMaker;
  MPScene         m_scene;

  FILE *in              = argc > 1 ? fopen(argv[1], "r") : NULL;
  const char  *m_prefix = argc > 2 ? argv[2] : "";
  int          nrQuery = argc > 3 ? atoi(argv[3]) : 1;

  if (argc > 1)
  {
    m_scene.SetupFromFile(in);
    m_scene.CompleteSetup();
    fclose(in);
  }
  m_queryMaker.SetScene(&m_scene);
  m_queryMaker.GenerateQueries(1,nrQuery,m_prefix);

  return 0;
}
