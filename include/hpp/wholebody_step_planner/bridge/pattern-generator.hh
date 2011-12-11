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
    typedef ChppGikSupportPolygon* supportPolygon_t;
    typedef ChppRobotMotion* robotMotion_t;
    typedef ChppRobotMotionSample motionSample_t;
    typedef Planner* planner_t;
    typedef ChppGikFootprint* footprint_t;

    // Container for a concatenation of whole-body robot trajectories
    // that form the entire dynamic trajectory.
    typedef std::vector<robotMotion_t> robotMotions_t;

    class PatternGenerator : public walk::PatternGenerator2d
    {
    public:
      explicit PatternGenerator (const planner_t& planner);

      virtual ~PatternGenerator ();

      const planner_t& planner () const;

      void planner (const planner_t& planner);

      void
      setRobotFootPosition (const bool isLeftFoot,
			    walk::HomogeneousMatrix3d& footPosition);

      void setRobotPosition (const vectorN& configuration,
			     walk::HomogeneousMatrix3d& leftFootPosition,
			     walk::HomogeneousMatrix3d& rightFootPosition,
			     walk::Vector3d& centerOfMassPosition,
			     walk::Posture& posture);
      
      void setInitialRobotPosition (const vectorN& configuration);

      void setFinalRobotPosition (const vectorN& configuration);

    protected:

      void computeTrajectories ();

    private:
      planner_t planner_;
    };
  } // end of namespace hpp.
} // end of namespace wholeBodyStepPlanner.

#endif
