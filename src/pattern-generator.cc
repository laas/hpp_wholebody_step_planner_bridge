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
    PatternGenerator::PatternGenerator (const planner_t& planner)
      : walk::PatternGenerator2d (),
	planner_(planner)
    {
    }
    
    PatternGenerator::~PatternGenerator ()
    {
    }
    
    const planner_t&
    PatternGenerator::planner () const
    {
      return planner_;
    }
    
    void
    PatternGenerator::planner (const planner_t& planner)
    {
      planner_ = planner;
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
	planner_->robot ()->robot ()->leftFoot ()
	  ->getAnklePositionInLocalFrame (anklePositionInFootFrame);
      else
	planner_->robot ()->robot ()->rightFoot ()
	  ->getAnklePositionInLocalFrame (anklePositionInFootFrame);

      walk::HomogeneousMatrix3d ankleTransformInFootFrame;
      walk::convertVector3dToTrans3d (ankleTransformInFootFrame,
				      anklePositionInFootFrame);

      matrix4d ankleCurrentTransformation;
      if (isLeftFoot)
	ankleCurrentTransformation = planner_->robot ()->robot ()->leftAnkle ()
	  ->currentTransformation ();
      else
	ankleCurrentTransformation = planner_->robot ()->robot ()->rightAnkle ()
	  ->currentTransformation ();

      walk::HomogeneousMatrix3d ankleTransformInAbsoluteFrame;
      walk::convertToTrans3d (ankleTransformInAbsoluteFrame,
			      ankleCurrentTransformation);

      footPosition = ankleTransformInAbsoluteFrame
	* ankleTransformInFootFrame.inverse ();
    }
    
    void
    PatternGenerator::setRobotPosition
    (const vectorN& configuration,
     walk::HomogeneousMatrix3d& leftFootPosition,
     walk::HomogeneousMatrix3d& rightFootPosition,
     walk::Vector3d& centerOfMassPosition,
     walk::Posture& posture)
    {
      planner_->robot ()->staticState (configuration);

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
			       planner_->robot ()->robot ()->positionCenterOfMass ());
      
      // Fill posture with additionnal dofs in upper body excluding the
      // free-flyer.
      vectorN ubMask = planner_->robot ()->maskFactory ()->upperBodyMask ();
      for (unsigned dofId = 0; dofId < 6; ++dofId)
	ubMask[dofId] = 0;

      unsigned activeDofNb = 0;
      for (unsigned dofId = 0; dofId < ubMask.size (); ++dofId)
	if (ubMask[dofId])
	  ++activeDofNb;

      // Resize posture to upper body dof numbers (size - 6 - 6 -6).
      posture.resize (ubMask.size () - 18);
      posture.setZero ();
      
      for (unsigned dofId = 18; dofId < ubMask.size (); ++dofId)
	posture[dofId - 18] = configuration[dofId];
    }
      
    void 
    PatternGenerator::setInitialRobotPosition (const vectorN& configuration)
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

    void
    PatternGenerator::setFinalRobotPosition (const vectorN& configuration)
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
    
    void
    PatternGenerator::computeTrajectories ()
    {
      using namespace boost::posix_time;

      // Fill initial posture information.
      vectorN initialConfiguration
	= planner_->robotMotions ()[0]->firstSample ()->configuration;
      setInitialRobotPosition (initialConfiguration);

      // Fill final posture information.
      vectorN finalConfiguration
	= planner_->robotMotions ()[planner_->robotMotions ().size () - 1]->lastSample ()
	->configuration;
      setFinalRobotPosition (finalConfiguration);
      
      // Fill trajectories information.
      for (unsigned motionId = 0; motionId < planner_->robotMotions ().size (); ++motionId)
	{
	  robotMotion_t robotMotion = planner_->robotMotions ()[motionId];
	  const ChppRobotMotionSample * motionSample
	    = robotMotion->firstSample ();

	  while (motionSample)
	    {
	      // Retrieve sample configuration and convert it to
	      // OpenHRP format.
	      std::vector<double>
		kineoCfg (planner_->humanoidRobot ()->countDofs ());
	      std::vector<double>
		openHrpCfg (planner_->humanoidRobot ()
			    ->countDofs ());
	      
	      planner_->humanoidRobot ()->jrlDynamicsToKwsDofValues
		(motionSample->configuration, kineoCfg);
	      planner_->kwsToOpenHrpDofValues (kineoCfg,
					      openHrpCfg);
	      vectorN sampleConfiguration (planner_->humanoidRobot ()
					   ->countDofs ());
	      for (unsigned i = 0;
		   i < planner_->humanoidRobot ()->countDofs ();
		   ++i)
		sampleConfiguration[i] = openHrpCfg[i];

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
		= milliseconds (robotMotion->samplingPeriod () * 1e3);
	      stampedSampleLeftFootPosition.position
		= sampleLeftFootPosition;
	      getLeftFootTrajectory ().data ()
		.push_back (stampedSampleLeftFootPosition);
 
	      walk::StampedPosition3d stampedSampleRightFootPosition;
	      stampedSampleRightFootPosition.duration
		= milliseconds(robotMotion->samplingPeriod () * 1e3);
	      stampedSampleRightFootPosition.position
		= sampleRightFootPosition;
	      getRightFootTrajectory ().data ()
		.push_back (stampedSampleRightFootPosition);

	      walk::StampedVector3d stampedSampleCenterOfMassPosition;
	      stampedSampleCenterOfMassPosition.duration
		= milliseconds(robotMotion->samplingPeriod () * 1e3);
	      stampedSampleCenterOfMassPosition.position
		= sampleCenterOfMassPosition;
	      getCenterOfMassTrajectory ().data ()
		.push_back (stampedSampleCenterOfMassPosition);

	      walk::StampedVectorNd stampedSamplePosture;
	      stampedSamplePosture.duration
		= milliseconds(robotMotion->samplingPeriod () * 1e3);
	      stampedSamplePosture.position
		= samplePosture;
	      getPostureTrajectory ().data ()
		.push_back (stampedSamplePosture);

	      // Fill ZMP trajectory.
	      walk::StampedVector2d stampedZMP;
	      stampedZMP.duration
		= milliseconds(robotMotion->samplingPeriod () * 1e3);
	      walk::convertToVector2d (stampedZMP.position,
				       motionSample->ZMPworPla);
	      getZmpTrajectory ().data ()
		.push_back (stampedZMP);

	      // Iterate motion sample.
	      motionSample = robotMotion->nextSample ();
	    }
	}
    }
  } // end of namespace hpp.
} // end of namespace wholeBodyStepPlanner.
