// Copyright (C) 2011 by Antonio El Khoury.
//

#ifndef HPP_WHOLEBODY_STEP_PLANNER_PATTERN_GENERATOR
# define HPP_WHOLEBODY_STEP_PLANNER_PATTERN_GENERATOR

# include <vector>

# include <hpp/gik/robot/standing-robot.hh>
# include <hpp/gik/robot/foot-print-related.hh>
# include <hpp/gik/robot/robot-motion.hh>

# include <hpp/wholebody-step-planner/planner.hh>

# include <walk_interfaces/pattern-generator.hh>

namespace hpp
{
  namespace wholeBodyStepPlanner
  {
    typedef ChppGikStandingRobot* robot_t;
    typedef ChppGikSupportPolygon* supportPolygon_t;
    typedef ChppRobotMotion* robotMotion_t;
    typedef ChppRobotMotionSample motionSample_t;
    typedef Planner* planner_t;
    typedef CkwsPathShPtr path_t;
    typedef ChppGikFootprint* footprint_t;

    // Container for a concatenation of whole-body robot trajectories
    // that form the entire dynamic trajectory.
    typedef std::vector<robotMotion_t> robotMotions_t;

    class PatternGenerator : public walk::PatternGenerator2d
    {
    public:
      explicit PatternGenerator (const robot_t& robot,
				 const robotMotions_t& robotMotions);

      virtual ~PatternGenerator ();

      const robot_t& robot () const;

      void robot (const robot_t& robot);

      const robotMotions_t& robotMotions () const;

      void robotMotions (const robotMotions_t& robotMotions);

      void
      setRobotFootPosition (const bool isLeftFoot,
			    walk::HomogeneousMatrix3d& footPosition);

      template <class T>
      void setRobotPosition (const T& configuration,
			     walk::HomogeneousMatrix3d& leftFootPosition,
			     walk::HomogeneousMatrix3d& rightFootPosition,
			     walk::Vector3d& centerOfMassPosition,
			     walk::Posture& posture);
      
      template <class T>
      void setInitialRobotPosition (const T& configuration);

      template <class T>
      void setFinalRobotPosition (const T& configuration);

      template <class T>
      void computeTrajectories ();

    private:
      robot_t robot_;

      robotMotions_t robotMotions_;

      planner_t planner_;
    };
  } // end of namespace hpp.
} // end of namespace wholeBodyStepPlanner.

#endif
