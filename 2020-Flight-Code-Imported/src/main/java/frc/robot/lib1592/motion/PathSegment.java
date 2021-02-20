package frc.robot.lib1592.motion;

import java.awt.geom.Point2D;

public interface PathSegment {
	
	//=== Generic Getters ===//
	MotionProfile getDriveProfile();
	MotionProfile getRotateProfile();
	Waypoint getEndPoint();
	Waypoint getStartPoint();
	
	/**
	 * Inertial angle of the robot's instantaneous velocity vector
	 * 
	 * @param t  abs time on path
	 * @return   velocity vector azimuth in degrees
	 */
	double getVelocityHeading(double t);
	
	/**
	 * Get Robot inertial 2D position
	 * @param t  abs time on path
	 * @return   <Point2D> position in length units
	 */
	Point2D getRobotPosition(double t);
	
	/**
	 * Get Robot's heading
	 * 
	 * @param t  abs time on path
	 * @return   robot body to inertial heading in degrees
	 */
	double getRobotHeading(double t);
	
	/**
	 * Get Rate of change of robot
	 * @param t  abs time on path
	 * @return   robot rotation rate in deg/s
	 */
	double getRobotHeadingRate(double t);
	
	/**
	 * @return  length of (along) the path in length units
	 */
	double getLength();
	
//	/**
//	 * Set the start times of the contained drive and rotate
//	 * motion profiles
//	 * 
//	 * @param t  absolute start time
//	 */
//	default void setStartTime(double t) {
//		getDriveProfile().setStartTime(t);
//		if (getRotateProfile() != null) getRotateProfile().setStartTime(t);
//	}
	
	/**
	 * Get the velocity vector of the robot
	 * @param t  time along the segment
	 * @return 2D velocity state
	 */
	default Point2D getRobotVelocity(double t) {
		TrajectoryPoint tp = getDriveTrajPoint(t);
		double cosAz = Math.cos(Math.toRadians(getVelocityHeading(t)));
		double sinAz = Math.sin(Math.toRadians(getVelocityHeading(t)));
		return new Point2D.Double(tp.vel() * cosAz, tp.vel() * sinAz);
	}

	/**
	 * Get the time to complete the path segment
	 * 
	 * @return time
	 */
	default double getDuration() {
		//Default is to treat drive profile as the primary
		double tStart = getDriveProfile().startTime();
		double tEndDrive = getDriveProfile().endTime();
		double tEndRotate = 0;
		if (getRotateProfile() != null)
			tEndRotate = getRotateProfile().endTime();
		double vEndDrive = getDriveProfile().endState().vel(); 
		//TODO: If drive profile comes to a stop, we can wait on rotation, but should we???
		//Ensures turn can complete on the final waypoint but may not be desirable in the middle of a path
		if (vEndDrive <= MotionUtil.EPSILON)
			tEndDrive = Math.max(tEndDrive,tEndRotate);				
		return tEndDrive - tStart;
	}
	
	/**
	 * Get the time first time on the path
	 * 
	 * @return  path relative time
	 */
	default double getStartTime() {
		//Rotation and Drive should always start at the same time
		return getDriveProfile().startTime();
	}
	
	/**
	 * Last time on the path
	 * @return  path relative time
	 */
	default double getEndTime() {
		return getStartTime() + getDuration();
	}

	/**
	 * Does specified time lie on this path segment
	 * 
	 * @param t  time in seconds
	 * @return  does specified time lie on the path segment
	 */
	default boolean containsTime(double t) {
		return t >= getStartTime() && t <= getEndTime();
	}
	
	/**
	 * Get the robot state at the final waypoint
	 * 
	 * @return state 	final waypoint expressed as a position and velocity state
	 */
	default RobotState getFinalState() {
		final double t = getEndTime();
		return getRobotState(t);
//		return new RobotState(getEndPoint().getX(), getEndPoint().getY(),
//				getEndPoint().magVelocity * Math.cos(Math.toRadians(getVelocityHeading(t))), 
//				getEndPoint().magVelocity * Math.sin(Math.toRadians(getVelocityHeading(t))),
//				getRobotHeading(t),getRobotHeadingRate(t),t);
	}
	
	/**
	 * Get the robot kinematic state along the path segment
	 * 
	 * @param t 		time along the segment
	 * @return state 	robot position and velocity state
	 */
	default RobotState getRobotState(double t) {
		Point2D pos = getRobotPosition(t);
		Point2D vel = getRobotVelocity(t);
		double psi = getRobotHeading(t);
		double psiDot = getRobotHeadingRate(t);
		return new RobotState(pos,vel,psi,psiDot,t);
	}
	
	/**
	 * Get the trajectory point of drive profile at time t along the PathSegment
	 * @param t 			time along the path segment
	 * @return trajPoint	1D kinematic state from the start point
	 */
	default TrajectoryPoint getDriveTrajPoint(double t) {
		return getDriveProfile().stateByTimeClamped(t);
	}
	
	/**
	 * Get the trajectory point of rotate profile at time t along the PathSegment
	 * @param t 			time along the path segment
	 * @return trajPoint	1D kinematic state from the start point
	 */
	default TrajectoryPoint getRotateTrajPoint(double t) {
		return getRotateProfile().stateByTimeClamped(t);
	}
}
