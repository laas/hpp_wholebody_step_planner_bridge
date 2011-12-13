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
    PatternGenerator::computeStepParameters
    (const hppFootprint_t hppFootprint,
     const double samplingPeriod,
     double& zmpEndShiftTime,
     double& zmpStartShiftTime,
     double& footFlightTime)
    {
      double stepFrac
	= planner_->stepFracOfFootprint ()[hppFootprint];
      zmpEndShiftTime = stepFrac * planner_->zmpEndShiftTime ();
      zmpStartShiftTime = stepFrac * planner_->zmpStartShiftTime ();
      footFlightTime = stepFrac * planner_->footFlightTime ();
      footFlightTime = ((int) (footFlightTime / samplingPeriod) +1)
	* samplingPeriod;
      zmpStartShiftTime
	= ((int) (zmpStartShiftTime / samplingPeriod) +1) * samplingPeriod;
      zmpEndShiftTime
	= ((int) (zmpEndShiftTime / samplingPeriod) +1) * samplingPeriod;
    }

    void
    PatternGenerator::computeFootprintsAndTrajectories ()
    {
      // Initialize empty footsteps sequence.
      PatternGenerator::footprints_t footprints;

      // Cycle through footprints provided by planner and build
      // footprint sequence.
      for (unsigned footprintsId = 0;
	   footprintsId < planner_->resultFootprints ().size ();
	   ++footprintsId)
	{
	  using namespace boost::posix_time;
	  using namespace boost::gregorian;

	  Planner::footprintOfParam_t hppFootprints
	    = planner_->resultFootprints ()[footprintsId];

	  std::map<double,double> paramOfTime = planner_->paramOfTime ();
	  std::map<double,double>::iterator paramOfTimeIt
	    = ++paramOfTime.begin ();

	  date motionStartDate = day_clock::local_day();
	  double samplingPeriod = 0.005;

	  for (Planner::footprintOfParam_t::iterator it
		 = hppFootprints.begin ();
	       it != hppFootprints.end ();
	       ++it)
	    {
	      hppFootprint_t hppFootprint1= (*it).second;
	      PatternGenerator::footprint_t footprint;

	      // Compute step parameters for footprint.
	      double zmpEndShiftTime1 = 0;
	      double zmpStartShiftTime1 = 0;
	      double footFlightTime1 = 0;

	      computeStepParameters (hppFootprint1,
				     samplingPeriod,
				     zmpEndShiftTime1,
				     zmpStartShiftTime1,
				     footFlightTime1);

	      // Fill footprint begin time and create Time object by
	      // adding offset to motion begin date.
	      //
	      // Time value in paramOfTime corresponds to the previous
	      // step end, including the zmpshift time to the center
	      // of the support polygon. The zmp end shift time hence
	      // needs to be subtracted.
	      walk::TimeDuration beginTimeFromStart
		= milliseconds
		(((* ++paramOfTimeIt).first - zmpEndShiftTime1)
		 * 1e3);

	      footprint.beginTime
		= walk::Time (motionStartDate, beginTimeFromStart);

	      // Fill footprint duration by adding up zmp shift times
	      // over three successive steps (during which the current
	      // foot is on the ground).
	      walk::TimeDuration duration = milliseconds (0);
	      duration += milliseconds (zmpEndShiftTime1 * 1e3);

	      ++it;
	      if (it != hppFootprints.end ())
		{
		  hppFootprint_t hppFootprint2 = (*it).second;

		  double zmpEndShiftTime2 = 0;
		  double zmpStartShiftTime2 = 0;
		  double footFlightTime2 = 0;

		  computeStepParameters (hppFootprint2,
					 samplingPeriod,
					 zmpEndShiftTime2,
					 zmpStartShiftTime2,
					 footFlightTime2);

		  duration += milliseconds ((zmpEndShiftTime2
					     + zmpStartShiftTime2
					     + footFlightTime2) * 1e3);

		  ++it;
		  if (it != hppFootprints.end ())
		    {
		      hppFootprint_t hppFootprint3 = (*it).second;

		      double zmpEndShiftTime3 = 0;
		      double zmpStartShiftTime3 = 0;
		      double footFlightTime3 = 0;

		      computeStepParameters (hppFootprint3,
					     samplingPeriod,
					     zmpEndShiftTime3,
					     zmpStartShiftTime3,
					     footFlightTime3);

		      duration += milliseconds (zmpStartShiftTime3 * 1e3);
		    }
		  --it;
		}
	      --it;

	      footprint.duration = duration;

	      // Fill footprint position.
	      footprint.position[0] = hppFootprint1->x ();
	      footprint.position[1] = hppFootprint1->y ();
	      footprint.position[2] = hppFootprint1->th ();

	      footprints.push_back (footprint);
	    }
	}

      // Set Footprints and trajectories. Start with right foot.
      setFootprints (footprints, false);
    }

    void
    PatternGenerator::setRobotFootPosition
    (const bool isLeftFoot,
     walk::HomogeneousMatrix3d& footPosition)
    {
      vector3d anklePositionInFootFrame;
      if (isLeftFoot)
	planner_->humanoidRobot ()->leftFoot ()
	  ->getAnklePositionInLocalFrame (anklePositionInFootFrame);
      else
	planner_->humanoidRobot ()->rightFoot ()
	  ->getAnklePositionInLocalFrame (anklePositionInFootFrame);

      walk::HomogeneousMatrix3d ankleTransformInFootFrame;
      walk::convertVector3dToTrans3d (ankleTransformInFootFrame,
				      anklePositionInFootFrame);

      matrix4d ankleCurrentTransformation;
      if (isLeftFoot)
	ankleCurrentTransformation = planner_->humanoidRobot ()
	  ->leftAnkle ()->currentTransformation ();
      else
	ankleCurrentTransformation = planner_->humanoidRobot ()
	  ->rightAnkle ()->currentTransformation ();

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
      planner_->humanoidRobot ()->hppSetCurrentConfig (configuration);

      setRobotFootPosition (true, leftFootPosition);

      setRobotFootPosition (false, rightFootPosition);

      walk::convertToVector3d (centerOfMassPosition,
			       planner_->humanoidRobot ()->positionCenterOfMass ());
      
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
      walk::Posture posture;

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
	= planner_->robotMotions ()[planner_->robotMotions ().size () - 1]
	->lastSample ()->configuration;
      setFinalRobotPosition (finalConfiguration);

      // Fill trajectories information.
      for (unsigned motionId = 0;
	   motionId < planner_->robotMotions ().size ();
	   ++motionId)
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
