/**
 * 
 */
package frc.robot.lib1592.motion;

import java.awt.geom.Point2D;

/**
 * @author ddyer
 *
 */
public class LinearPathSegment implements PathSegment{
	private RobotState mStart;
	private Waypoint mEnd;
	private MotionProfile mDriveProfile;
	private MotionProfile mRotateProfile = null;

	//Relative path
	private double dx;
	private double dy;
	//Velocity along the path
	private double vInit;

	/**
	 * Create linear path segment
	 * 
	 * @param start  first Waypoint
	 * @param end    final Waypoint
	 */
	public LinearPathSegment(RobotState start, Waypoint end) {
		//=== Drive ===//
		mStart = start;
		mEnd = end;
		dx = mEnd.getX() - start.getX();
		dy = mEnd.getY() - start.getY();
		//Velocity along the path
		//TODO: warn if velocity is significantly off the path?
		vInit = (start.xVelocity*dx + start.yVelocity*dy) / getLength(); //dot produce with unit path unit vector
		//1D Goal is relative length along the segment
		MotionProfileGoal goal = new MotionProfileGoal(getLength(),mEnd.magVelocity,mEnd.driveConstraints.endBehavior);
		//Init pos and time start at zero because they are relative to the segment
		TrajectoryPoint init = new TrajectoryPoint(start.time,0,vInit,0); //NOTE: init accel doesn't matter for Trapezoid
		mDriveProfile = TrapezoidProfileGenerator.generateProfile(mEnd.driveConstraints.maxAbsVelocity,mEnd.driveConstraints.maxAbsAccel,goal,init);

		//=== Turning ===//
		//If heading is NaN, just make the robot turn to align with the velocity vector (skid steering)
		if (Double.isFinite(mEnd.heading) && mEnd.rotateConstraints != null) {
			//Extract constraints
			TrajConstraints constraints = mEnd.rotateConstraints.clone();
			//1D Goal is relative length along the segment
			double dTheta = MotionUtil.getContinuousError(mEnd.heading - mStart.yaw, 360d);
			goal = new MotionProfileGoal(dTheta,mEnd.magHeadingRate,constraints.endBehavior);
			//Init pos and time start at zero because they are relative to the segment
			init = new TrajectoryPoint(start.time,0,start.yawRate,0); //NOTE: init accel doesn't matter for Trapezoid
			mRotateProfile = TrapezoidProfileGenerator.generateProfile(constraints.maxAbsVelocity,constraints.maxAbsAccel,goal,init);

			double durationTurn = mRotateProfile.duration();
			double durationDrive = mDriveProfile.duration();
			if (MotionUtil.epsEquals(durationTurn, durationDrive, MotionUtil.SAME_TIME)); //If close, just run as is
			else if (durationDrive > durationTurn) {
				switch (mEnd.rotateBehavior) {
				case EARLY:
					//This is really default
					break;
				case LATE:
					//Shift the start time of each profile enough later to finsh at the same time as drive
					//FIXME: this only works if the start velocity is 0.  Need a more general soln
//					mRotateProfile.timeShift(durationDrive-durationTurn);
//					break;

				case MIN_VEL:
					//TODO
					//break;

				case MIN_ACCEL:
					//TODO
					//break;

				default:
					System.err.println("Unimplemented Mode");
					break;
				}
			}
			else {
				//TODO: do we hold the drive profile to finish the turn or violate some constraint?
				//For now, do nothing.  This means the rotate profile will just not finish
			}
		}
		
	}

	/**
	 * Get the robot position on the path segment
	 * 
	 * @param t 	time along the segment
	 * @return  	2D position state
	 */
	public Point2D getRobotPosition(double t) {
		TrajectoryPoint tp = getDriveTrajPoint(t);
		double cosAz = Math.cos(Math.toRadians(getVelocityHeading(t)));
		double sinAz = Math.sin(Math.toRadians(getVelocityHeading(t)));
		return new Point2D.Double(mStart.getX() + tp.pos*cosAz,
				mStart.getY() + tp.pos*sinAz);
	}


	/**
	 * Get the length of the path segment
	 * 
	 * @return length
	 */
	@Override
	public double getLength() {
		return Math.sqrt(dx*dx + dy*dy);
	}

	@Override
	public double getVelocityHeading(double t) {
		return Math.toDegrees(Math.atan2(dy,dx));
	}

	/**
	 * Get the motion profile
	 * 
	 * @return motionProfile
	 */
	@Override
	public MotionProfile getDriveProfile() {
		return mDriveProfile;
	}

	@Override
	public Waypoint getEndPoint() {
		return mEnd;
	}

	@Override
	public Waypoint getStartPoint() {
		return new Waypoint(mStart.getX(), mStart.getY(), mEnd.driveConstraints,vInit,mStart.yaw,mEnd.rotateConstraints);
	}

	@Override
	public double getRobotHeading(double t) {
		if (mRotateProfile == null) {
			return getVelocityHeading(t);
		}
		TrajectoryPoint tp = getRotateTrajPoint(t);
		return mStart.yaw + tp.pos;
	}

	@Override
	public double getRobotHeadingRate(double t) {
		if (mRotateProfile == null) {
			//Drive straight line
			return 0;
		}
		TrajectoryPoint tp = getRotateTrajPoint(t);
		return tp.vel;
	}

	@Override
	public MotionProfile getRotateProfile() {
		return mRotateProfile;
	}
}
