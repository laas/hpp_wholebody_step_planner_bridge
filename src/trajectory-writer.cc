#include <iostream>
#include <fstream>

#include <KineoModel/kppUserData.h>
#include <KineoWorks2/kwsUtility.h>
#include <KineoModel/kppLicense.h>

#include <kwsIO/kwsioPath.h>
#include <kwsIO/kwsioConfig.h>
#include <hpp/wholebody-step-planner/planner.hh>

#include <walk_interfaces/yaml.hh>

#include "hpp/wholebody_step_planner/bridge/pattern-generator.hh"

#include "load-hrp2.cc"
#include "path-parser.cc"

int main(int argc, char *argv[])
{
  using namespace hpp::wholeBodyStepPlanner;

  // Check arguments number. Should be 1 if only path is given, and 2 if environment and path (in that order) are given.
  if (argc != 2 && argc != 3)
    {
      std::cerr	<< "Wrong arguments given, expected 1 (path to path.kxml) or 2 (path to env.kxml and path.kxml)" << std::endl;
      return -1;
    }

  // Check license.
  if ( !CkppLicense::initialize() )
    {
      std::cerr << "Error: license for KPP SDK was not found" << std::endl;
      return -1;
    }
  else
    {
      std::cout << "license for KPP SDK was found" << std::endl;
    }

  CkppUserData::getInstance()->initializeParameters();
  CkwsUtility::randomSeed (time (NULL));

  // Create planner instance and set footprint limits.
  Planner* planner = new Planner (0.05);
  
  planner->setFootPrintLimits(-0.17,0.17,-0.25,-0.13,-M_PI /4,0.1);

  // Build HRP-2 robot and load it in planner.
  ChppHumanoidRobotShPtr robot;
  if (loadHrp2Model (robot) !=  KD_OK)
      std::cerr << "Planner::initScene(): error in loading HRP2"
		<< std::endl;
  
  planner->addHppProblem(robot,0.1);

  // Initialize planner.

  planner->initializeProblem ();

  // Load environment from file given in first argument if two
  // arguments are given.

  if (argc == 3)
    if (KD_OK != planner->parseFile (argv[1]))
      {
	std::cerr << "Planner::parseFile: could not load environment."
		  << std::endl;
	std::cerr << "Make sure that the environment path is correctly set."
		  << std::endl;
	return -1;
      }
  
  // Load path to animate from file given in first or second argument.
  CkwsPathShPtr path = CkwsPath::create (planner->humanoidRobot ());
  getKineoPathFromFile (argv[argc - 1], path);

  // Remove constraint that constrains feet to the ground before
  // starting animation.

  planner->humanoidRobot ()->userConstraints ()
    ->remove (planner->wholeBodyConstraint ());

  // Animate Path.

  planner->findDynamicPath (path);

  // Create instance of pattern generator.

  PatternGenerator patternGenerator (planner);

  assert (!!patternGenerator.planner ());
  assert (patternGenerator.planner ()->robotMotions ().size () != 0);
  assert (!!patternGenerator.planner ()->robotMotions ()[0]);
  assert (!patternGenerator.planner ()->robotMotions ()[0]->empty ());

  // Compute trajectories in pattern generator using planner motion.

  patternGenerator.computeFootprintsAndTrajectories ();

  // Create yaml writer and export pattern generator with trajectories.

  walk::YamlWriter<PatternGenerator> yamlWriter (patternGenerator);

  yamlWriter.write ("./walk.yaml");
  
  return 0;
}
