#include <sstream>
#include <stdexcept>
#include <boost/bind.hpp>
#include <boost/date_time.hpp>

#include <Eigen/LU>

#include <LinearMath/btMatrix3x3.h>
#include <LinearMath/btQuaternion.h>

#include <ros/ros.h>

#include <walk_interfaces/yaml.hh>

#include <geometry_msgs/Pose.h>
#include <visualization_msgs/MarkerArray.h>
#include <walk_msgs/Footprint2d.h>
#include <walk_msgs/GetPath.h>

#include <walk_msgs/conversion.hh>

#include <KineoModel/kppUserData.h>
#include <KineoWorks2/kwsUtility.h>
#include <KineoModel/kppLicense.h>

#include <kwsIO/kwsioPath.h>
#include <kwsIO/kwsioConfig.h>
#include <hpp/wholebody-step-planner/planner.hh>

#include "hpp/wholebody_step_planner/bridge/pattern-generator.hh"

#include "../load-hrp2.cc"
#include "../path-parser.cc"

using namespace hpp::wholeBodyStepPlanner;

void convertFootprint(PatternGenerator::footprints_t& dst,
		      const std::vector<walk_msgs::Footprint2d>& src);

void convertFootprint(PatternGenerator::footprints_t& dst,
		      const std::vector<walk_msgs::Footprint2d>& src)
{
  using boost::posix_time::seconds;
  using boost::posix_time::milliseconds;

  dst.clear();
  std::vector<walk_msgs::Footprint2d>::const_iterator it = src.begin();
  for (; it != src.end(); ++it)
    {
      PatternGenerator::footprint_t footprint;
      footprint.beginTime = (it->beginTime).toBoost();
      footprint.duration =
	seconds(it->duration.sec) + milliseconds(it->duration.nsec * 1000);
      footprint.position(0) = it->x;
      footprint.position(1) = it->y;
      footprint.position(2) = it->theta;
      dst.push_back(footprint);
    }
}


using walk::HomogeneousMatrix3d;
using walk::Posture;

using walk_msgs::convertPoseToHomogeneousMatrix;
using walk_msgs::convertPointToVector3d;
using walk_msgs::convertTrajectoryToPath;
using walk_msgs::convertTrajectoryV3dToPath;
using walk_msgs::convertTrajectoryV2dToPath;

class GeneratorNode
{
public:
  explicit GeneratorNode(int argc, char **argv);
  ~GeneratorNode();

  void spin();

protected:
  bool getPath(walk_msgs::GetPath::Request&,
	       walk_msgs::GetPath::Response&);

private:
  /// \brief Main node handle.
  ros::NodeHandle nodeHandle_;

  ros::ServiceServer getPathSrv_;

  std::string frameName_;

  PatternGenerator patternGenerator_;

  visualization_msgs::MarkerArray footprints_;
  nav_msgs::Path leftFootPath_;
  nav_msgs::Path rightFootPath_;
  nav_msgs::Path comPath_;
  nav_msgs::Path zmpPath_;

  ros::Publisher footprintsPub_;
  ros::Publisher leftFootPub_;
  ros::Publisher rightFootPub_;
  ros::Publisher comPub_;
  ros::Publisher zmpPub_;

  int argc_;
  char **argv_;
};

GeneratorNode::GeneratorNode(int argc, char **argv)
  : nodeHandle_("halfsteps_pattern_generator"),
    getPathSrv_(),
    frameName_("/world"),
    patternGenerator_(),

    footprints_ (),
    leftFootPath_ (),
    rightFootPath_ (),
    comPath_ (),
    zmpPath_ (),

    footprintsPub_ (),
    leftFootPub_ (),
    rightFootPub_ (),
    comPub_ (),

    argc_ (argc),
    argv_ (argv)
{
  typedef boost::function<bool (walk_msgs::GetPath::Request&,
				walk_msgs::GetPath::Response&)> callback_t;
  callback_t callback = boost::bind(&GeneratorNode::getPath, this, _1, _2);
  getPathSrv_ = nodeHandle_.advertiseService("getPath", callback);

  footprintsPub_ = nodeHandle_.advertise<visualization_msgs::MarkerArray> ("footprints", 5);
  leftFootPub_ = nodeHandle_.advertise<nav_msgs::Path> ("left_foot", 5);
  rightFootPub_ = nodeHandle_.advertise<nav_msgs::Path> ("right_foot", 5);
  comPub_ = nodeHandle_.advertise<nav_msgs::Path> ("com", 5);
  zmpPub_ = nodeHandle_.advertise<nav_msgs::Path> ("zmp", 5);

  // Check arguments number. Should be 1 if only path is given, and 2 if environment and path (in that order) are given.
  if (argc_ != 2 && argc_ != 3)
    {
      std::cerr	<< "Wrong arguments given, expected 1 (path to path.kxml) or 2 (path to env.kxml and path.kxml)" << std::endl;
    }

  // Check license.
  if ( !CkppLicense::initialize() )
    {
      std::cerr << "Error: license for KPP SDK was not found" << std::endl;
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

  if (argc_ == 3)
    if (KD_OK != planner->parseFile (argv_[1]))
      {
	std::cerr << "Planner::parseFile: could not load environment."
		  << std::endl;
	std::cerr << "Make sure that the environment path is correctly set."
		  << std::endl;
      }
  
  // Load path to animate from file given in first or second argument.
  CkwsPathShPtr path = CkwsPath::create (planner->humanoidRobot ());
  getKineoPathFromFile (argv_[argc_ - 1], path);

  // Remove constraint that constrains feet to the ground before
  // starting animation.

  planner->humanoidRobot ()->userConstraints ()
    ->remove (planner->wholeBodyConstraint ());

  // Animate Path.

  planner->findDynamicPath (path);

  // Create instance of pattern generator.

  patternGenerator_.planner (planner);

  assert (!!patternGenerator_.planner ());
  assert (patternGenerator_.planner ()->robotMotions ().size () != 0);
  assert (!!patternGenerator_.planner ()->robotMotions ()[0]);
  assert (!patternGenerator_.planner ()->robotMotions ()[0]->empty ());

  // Compute trajectories in pattern generator using planner motion.

  patternGenerator_.computeFootprintsAndTrajectories ();
}

GeneratorNode::~GeneratorNode()
{}

void
GeneratorNode::spin()
{
  ros::Rate rate(10);

  while (ros::ok ())
    {
      footprintsPub_.publish (footprints_);
      leftFootPub_.publish (leftFootPath_);
      rightFootPub_.publish (rightFootPath_);
      comPub_.publish (comPath_);
      zmpPub_.publish (zmpPath_);

      ros::spinOnce();
      rate.sleep ();
    }
}

bool
GeneratorNode::getPath(walk_msgs::GetPath::Request& req,
		       walk_msgs::GetPath::Response& res)
{
  res.path.left_foot.header.seq = 0;
  res.path.left_foot.header.stamp.sec = 0;
  res.path.left_foot.header.stamp.nsec = 0;
  res.path.left_foot.header.frame_id = frameName_;

  res.path.right_foot.header = res.path.left_foot.header;
  res.path.center_of_mass.header = res.path.left_foot.header;
  res.path.zmp.header = res.path.left_foot.header;

  convertTrajectoryToPath(res.path.left_foot,
			  patternGenerator_.leftFootTrajectory(),
			  frameName_);
  convertTrajectoryToPath(res.path.right_foot,
			  patternGenerator_.rightFootTrajectory(),
			  frameName_);
  convertTrajectoryV3dToPath(res.path.center_of_mass,
			     patternGenerator_.centerOfMassTrajectory(),
			     frameName_);
  convertTrajectoryV2dToPath(res.path.zmp,
			     patternGenerator_.zmpTrajectory(),
			     frameName_);

  // Prepare topics data.
  leftFootPath_ = res.path.left_foot;
  leftFootPath_.header.frame_id = "/world";
  rightFootPath_ = res.path.right_foot;
  rightFootPath_.header.frame_id = "/world";
  convertTrajectoryV3dToPath(comPath_,
			     patternGenerator_.centerOfMassTrajectory(),
			     frameName_);
  comPath_.header.frame_id = "/world";
  convertTrajectoryV2dToPath(zmpPath_,
			     patternGenerator_.zmpTrajectory(),
			     frameName_);
  zmpPath_.header.frame_id = "/world";

  uint32_t shape = visualization_msgs::Marker::CUBE;
  uint32_t id = 0;
  bool isLeft = true;
  BOOST_FOREACH (const PatternGenerator::footprint_t& footprint,
		 patternGenerator_.footprints ())
    {
      visualization_msgs::Marker marker;
      // Set the frame ID and timestamp.  See the TF tutorials for
      // information on these.
      marker.header.frame_id = frameName_;
      marker.header.stamp = ros::Time::now();

      // Set the namespace and id for this marker.  This serves to
      // create a unique ID Any marker sent with the same namespace
      // and id will overwrite the old one
      marker.ns = "halfsteps_pattern_generator";
      marker.id = id++;

      // Set the marker type.
      marker.type = shape;

      // Set the marker action.
      marker.action = visualization_msgs::Marker::ADD;

      // Set the pose of the marker.  This is a full 6DOF pose
      // relative to the frame/time specified in the header
      marker.pose.position.x = footprint.position[0];
      marker.pose.position.y = footprint.position[1];
      marker.pose.position.z = 0.;

      btQuaternion quaternion;
      quaternion.setEuler (0., 0., footprint.position[2]);
      marker.pose.orientation.x = quaternion.x ();
      marker.pose.orientation.y = quaternion.y ();
      marker.pose.orientation.z = quaternion.z ();
      marker.pose.orientation.w = quaternion.w ();

      // Set the scale of the marker
      marker.scale.x = 0.2172;
      marker.scale.y = 0.138;
      marker.scale.z = 0.001;

      // Set the color
      marker.color.r = isLeft ? 1.0f : 0.0f;
      marker.color.g = isLeft ? 0.0f : 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 0.5f;

      marker.lifetime = ros::Duration();

      isLeft = !isLeft;

      footprints_.markers.push_back (marker);
    }

  std::stringstream ss;
  walk::YamlWriter<PatternGenerator> writer (patternGenerator_);
  writer.write (ss);
  nodeHandle_.setParam ("walk_movement", ss.str ());
  return true;
}



int main(int argc, char **argv)
{
  try
    {
      ros::init(argc, argv, "halfsteps_pattern_generator");
      GeneratorNode node (argc, argv);
      if (ros::ok())
	node.spin();
    }
  catch (std::exception& e)
    {
      std::cerr << "fatal error: " << e.what() << std::endl;
      ROS_ERROR_STREAM("fatal error: " << e.what());
      return 1;
    }
  catch (...)
    {
      ROS_ERROR_STREAM("unexpected error");
      return 2;
    }
  return 0;
}

