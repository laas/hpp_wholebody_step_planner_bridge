// Copyright (C) 2011 by Antonio El Khoury.
//

#include <Eigen/LU>

#include <walk_interfaces/types.hh>
#include <walk_interfaces/util.hh>

#include <hpp/wholebody_step_planner/bridge/pattern-generator.hh>

namespace hpp
{
  namespace wholeBodyStepPlanner
  {
    PatternGenerator::PatternGenerator (const robot_t& robot,
					const robotMotions_t& robotMotions,
					const planner_t& planner)
      : walk::PatternGenerator2d (),
	robot_ (robot),
	robotMotions_ (robotMotions),
	planner_(planner)
    {
    }
    
    PatternGenerator::~PatternGenerator ()
    {
    }
    
    const robot_t& 
    PatternGenerator::robot () const
    {
      return robot_;
    }

    void
    PatternGenerator::robot (const robot_t& robot)
    {
      robot_ = robot;
    }

    const robotMotions_t&
    PatternGenerator::robotMotions () const
    {
      return robotMotions_;
    }
    
    void
    PatternGenerator::robotMotions (const robotMotions_t& robotMotions)
    {
      robotMotions_ = robotMotions;
    }

    void
    PatternGenerator::setInitAndGoalConfig (const path_t& path)
    {
      planner_->initAndGoalConfig (path);
      planner_->initializeProblem ();
    }

    void
    PatternGenerator::planTrajectories ()
    {
      if (KD_OK == planner_->solveOneProblem (0))
	if (KD_OK == planner_->findDynamicPath
	    (planner_->getPath (0, planner_->getNbPaths (0) - 1)))
	  {
	    // Initialize empty footsteps sequence.
	    walk::footsteps_t footsteps ();

	    // Cycle through footprints provide by planner and build
	    // footsteps sequence.
	    for (unsigned footprintOfParamId = 0;
		 footprintOfParamId < planner_->resultFootprints_.size ();
		 ++footprintOfParamId)
	      {
		footprintOfParam_t footprintOfParam
		  = planner_->resultFootprints_[footprintOfParamId];
		
		for (footprintOfParam_t::iterator it = footprintOfParam.begin ();
		     it != footprintOfParam.end ();
		     ++it)
		  {
		    footprint_t footprint = (*it).second;
		    walk::footstep_t footstep;
		  }
	      }
	    setSteps (footsteps, false);
	  }
    }

    void
    PatternGenerator::setRobotFootPosition
    (const bool isLeftFoot,
     walk::HomogeneousMatrix3d& footPosition)
    {
      vector3d anklePositionInFootFrame;
      if (isLeftFoot)
	robot_->robot ()->leftFoot ()
	  ->getAnklePositionInLocalFrame (anklePositionInFootFrame);
      else
	robot_->robot ()->rightFoot ()
	  ->getAnklePositionInLocalFrame (anklePositionInFootFrame);

      walk::HomogeneousMatrix3d ankleTransformInFootFrame;
      walk::convertVector3dToTrans3d (ankleTransformInFootFrame,
				      anklePositionInFootFrame);

      matrix4d ankleCurrentTransformation;
      if (isLeftFoot)
	ankleCurrentTransformation = robot_->robot ()->leftAnkle ()
	  ->currentTransformation ();
      else
	ankleCurrentTransformation = robot_->robot ()->rightAnkle ()
	  ->currentTransformation ();

      walk::HomogeneousMatrix3d ankleTransformInAbsoluteFrame;
      walk::convertToTrans3d (ankleTransformInAbsoluteFrame,
			      ankleCurrentTransformation);

      footPosition = ankleTransformInAbsoluteFrame
	* ankleTransformInFootFrame.inverse ();
    }
    
    template <class T>
    void
    PatternGenerator::setRobotPosition
    (const T& configuration,
     walk::HomogeneousMatrix3d& leftFootPosition,
     walk::HomogeneousMatrix3d& rightFootPosition,
     walk::Vector3d& centerOfMassPosition,
     walk::Posture& posture)
    {
      robot_->staticState (configuration);

      setRobotFootPosition (true, leftFootPosition);

      setRobotFootPosition (false, rightFootPosition);

      // walk::trans2dToTrans3d (leftFootPosition,
      // 			      supportPolygon->leftFootprint ()->x (),
      // 			      supportPolygon->leftFootprint ()->y (),
      // 			      supportPolygon->leftFootprint ()->th ());
      
      // walk::trans2dToTrans3d (rightFootPosition,
      // 			      supportPolygon->rightFootprint ()->x (),
      // 			      supportPolygon->rightFootprint ()->y (),
      // 			      supportPolygon->rightFootprint ()->th ());

      walk::convertToVector3d (centerOfMassPosition,
			       robot_->robot ()->positionCenterOfMass ());
      
      // Fill posture with additionnal dofs in upper body excluding the
      // free-flyer.
      T ubMask = robot_->maskFactory ()->upperBodyMask ();

      posture.resize (ubMask.size ());
      posture.setZero ();
      
      for (unsigned dofId = 6; dofId < ubMask.size (); ++dofId)
	if (ubMask[dofId])
	  posture[dofId] = configuration[dofId];
    }
      
    template <class T>
    void 
    PatternGenerator::setInitialRobotPosition (const T& configuration)
    {
      walk::HomogeneousMatrix3d leftFootPosition;
      walk::HomogeneousMatrix3d rightFootPosition;
      walk::Vector3d centerOfMassPosition;
      walk::Posture posture (configuration.size ());

      setRobotPosition (configuration,
			leftFootPosition,
			rightFootPosition,
			centerOfMassPosition,
			posture);
      
      walk::PatternGenerator2d::setInitialRobotPosition 
	(leftFootPosition,
	 rightFootPosition,
	 centerOfMassPosition,
	 posture);
    }

    template <class T>
    void
    PatternGenerator::setFinalRobotPosition (const T& configuration)
    {
      walk::HomogeneousMatrix3d leftFootPosition;
      walk::HomogeneousMatrix3d rightFootPosition;
      walk::Vector3d centerOfMassPosition;
      walk::Posture posture (configuration.size ());

      setRobotPosition (configuration,
			leftFootPosition,
			rightFootPosition,
			centerOfMassPosition,
			posture);
      
      walk::PatternGenerator2d::setFinalRobotPosition(leftFootPosition,
						      rightFootPosition,
						      centerOfMassPosition,
						      posture);
    }
    
    template <class T>
    void
    PatternGenerator::computeTrajectories ()
    {
      // Fill initial posture information.
      const T initialConfiguration
	= robotMotions_[0]->firstSample ()->configuration;
      setInitialRobotPosition (initialConfiguration);

      // Fill final posture information.
      const T finalConfiguration
	= robotMotions_[robotMotions_.size () - 1]->lastSample ()
	->configuration;
      setFinalRobotPosition (finalConfiguration);
      
      // Fill trajectories information.
      for (unsigned motionId = 0; motionId < robotMotions_.size (); ++motionId)
	{
	  robotMotion_t robotMotion = robotMotions_[motionId];

	  for (double sampleTime = robotMotion->startTime ();
	       sampleTime < robotMotion->endTime ();
	       sampleTime += robotMotion->samplingPeriod ())
	    {
	      // Retrieve motion sample and sample configuration.
	      motionSample_t motionSample;
	      robotMotion->getSampleAtTime (sampleTime, motionSample);

	      const T sampleConfiguration = motionSample.configuration;
	      
	      // Retrieve foot, CoM, and posture values for sample.
	      walk::HomogeneousMatrix3d sampleLeftFootPosition;
	      walk::HomogeneousMatrix3d sampleRightFootPosition;
	      walk::Vector3d sampleCenterOfMassPosition;
	      walk::Posture samplePosture;

	      setRobotPosition (sampleConfiguration,
				sampleLeftFootPosition,
				sampleRightFootPosition,
				sampleCenterOfMassPosition,
				samplePosture);
	      
	      // Fill foot, CoM, and posture trajectories.
	      walk::StampedPosition3d stampedSampleLeftFootPosition;
	      stampedSampleLeftFootPosition.duration
		= robotMotion->samplingPeriod ();
	      stampedSampleLeftFootPosition.position
		= sampleLeftFootPosition;
	      leftFootTrajectory ().data ()
		.push_back (stampedSampleLeftFootPosition);
 
	      walk::StampedPosition3d stampedSampleRightFootPosition;
	      stampedSampleRightFootPosition.duration
		= robotMotion->samplingPeriod ();
	      stampedSampleRightFootPosition.position
		= sampleRightFootPosition;
	      rightFootTrajectory ().data ()
		.push_back (stampedSampleRightFootPosition);

	      walk::StampedVector3d stampedSampleCenterOfMassPosition;
	      stampedSampleCenterOfMassPosition.duration
		= robotMotion->samplingPeriod ();
	      stampedSampleCenterOfMassPosition.position
		= sampleCenterOfMassPosition;
	      centerOfMassTrajectory ().data ()
		.push_back (stampedSampleCenterOfMassPosition);

	      walk::StampedVectorNd stampedSamplePosture;
	      stampedSamplePosture.duration
		= robotMotion->samplingPeriod ();
	      stampedSamplePosture.position
		= samplePosture;
	      postureTrajectory ().data ()
		.push_back (stampedSamplePosture);

	      // Fill ZMP trajectory.
	      walk::StampedVector2d stampedZMP;
	      stampedZMP.duration
		= robotMotion->samplingPeriod ();
	      walk::convertToVector2d (stampedZMP.position,
				       motionSample.ZMPworPla);
	      zmpTrajectory ().data ()
		.push_back (stampedZMP);
	    }
	}
    }
  } // end of namespace hpp.
} // end of namespace wholeBodyStepPlanner.
