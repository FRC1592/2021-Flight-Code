package frc.robot.lib1592.motion;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Point2D.Double;

import frc.robot.lib1592.motion.MotionProfileGoal.CompletionBehavior;

public class CurvedPathSegment implements PathSegment{

	@SuppressWarnings("unused")
	private RobotState mStart;
	private Waypoint mEnd;
	private MotionProfile mDriveProfile;
	private MotionProfile mRotateProfile = null;
	private double mRadius;
	private Waypoint mTangent1;
	private Waypoint mTangent2;

	//Center point of the arc
	private Point2D mPivot;
	//Old waypiont we just bypassed on the arc
	@SuppressWarnings("unused")
	private Point2D mOldWaypoint;
	//Threshold for comparing to zero
	private final double EPSILON = 0.0001;
	//Initial Angle from the pivot to the first tangent point
	private final double mInitialVelocityHeading;  //rad
	//Direction to rotate around the pivot
	private boolean isRotationPositive = true;
	//Included angle at middle waypoint
	private final double mInternalAngle;  //[rad]
	//Initial radial unit vector
	private final double mInitRadialAngle;  //[rad]
	//Initial heading
	//TODO: do we need to do this profile relative?  Can we just eliminate this?
	private double mInitHeading;

	public CurvedPathSegment(RobotState start, Waypoint mid, Waypoint end) {
		//=== Capture Inputs ===
		mStart = start;
		mOldWaypoint = mid;
		mEnd = end;
		//Get smoothing radius from the old waypoint
		mRadius = mid.smoothingRadius;
		//Capture for use in setting time at mTangent1 (curved start)
		double startTime = start.time;

		//=== vectors from mid to start and end ===
		Line2D lineP2A = new Line2D.Double(start,mid);
		double distP2A = mid.distance(start);
		double dxP2A = start.getX() - mid.getX();
		double dyP2A = start.getY() - mid.getY();
		Line2D lineP2B = new Line2D.Double(end,mid);
		double distP2B = mid.distance(end);
		double dxP2B = end.getX() - mid.getX();
		double dyP2B = end.getY() - mid.getY();

		//=== Angle Between AP and BP vectors ===
		mInternalAngle = MotionUtil.getInternalAngle(start, mid, end);

		//=== Distance from tangent points to old Waypoint, mid ===
		double l = mRadius * Math.tan((Math.PI-mInternalAngle)/2);
		//Can't let l be greater than either of the old segment lengths
		if (l > Math.min(start.distance(mid), end.distance(mid))) {
			System.err.println("Radius too large for segment.  Truncating...");
			l = Math.min(start.distance(mid), end.distance(mid));
			mRadius = l / Math.tan((Math.PI-mInternalAngle)/2);
		}

		//Compute trajectory constraints for Tangent 2.  Mainly the same as end Waypoint
		TrajConstraints t2Constraints = end.driveConstraints.clone();
		//Make vmax the average of the 1st and 2nd segments
		t2Constraints.maxAbsVelocity = (mid.driveConstraints.maxAbsVelocity + t2Constraints.maxAbsVelocity) / 2;

		//=== Check radial acceleration ===
		double ar = t2Constraints.maxAbsVelocity * t2Constraints.maxAbsVelocity / mRadius;
		if (ar > t2Constraints.maxAbsAccel) {
			System.err.println("Potentially high radial accel = " + ar);
			//			t2Constraints.maxAbsVelocity = Math.sqrt(t2Constraints.maxAbsAccel * mRadius);
		}

		//Initial vel vector is along the vector from start to mid (-P2A)
		double vxUnit = -dxP2A / distP2A;
		double vyUnit = -dyP2A / distP2A;

		//=== Find the tangent points ===
		//Old waypoint + distance, l, along a unit vector
		Point2D T1 = new Point2D.Double(mid.getX() + dxP2A/distP2A * l, 
				mid.getY() + dyP2A/distP2A *l);
		if (!isPntOnLine(T1,lineP2A)) {
			T1 = new Point2D.Double(mid.getX() - dxP2A/distP2A * l,
					mid.getY() - dyP2A/distP2A * l);
		}
		Point2D T2 = new Point2D.Double(mid.getX() + dxP2B/distP2B * l,
				mid.getY() + dyP2B/distP2B * l);
		if (!isPntOnLine(T2,lineP2B)) {
			T2 = new Point2D.Double(mid.getX() - dxP2B/distP2B * l,
					mid.getY() - dyP2B/distP2B * l);
		}
		//T1 and T2 become new starting/ending Waypoints with constraints from mid
		//TODO: warn if velocity is significantly off the path?
		double v0 = start.xVelocity*vxUnit + start.yVelocity*vyUnit ;
		double vf1 = Math.sqrt(2*mid.driveConstraints.maxAbsAccel*start.distance(T1) + v0*v0);
		mTangent1 = new Waypoint(T1.getX(),T1.getY(),mid.driveConstraints,vf1,mid.heading,mid.rotateConstraints); //Waypoint ensures vf is within constraints
		//Need to avoid ADJUST_ACCEL for tangent points in-case they're very close to the previous causing snap turns
		mTangent1.rotateConstraints.endBehavior = CompletionBehavior.ADJUST_VELOCITY;
		//Compute final velocity of the segment to ensure we can stop before the next waypoint
		//vf = sqrt((d-l)*2*a + end.vf^2)
		double vf2 = Math.sqrt((end.distance(mid)-l)*2*end.driveConstraints.maxAbsAccel + end.magVelocity*end.magVelocity);
		mTangent2 = new Waypoint(T2.getX(),T2.getY(),t2Constraints,vf2,end.heading,end.rotateConstraints);  //Waypoint ensures vf is within constraints
		//Need to avoid ADJUST_ACCEL for tangent points in-case they're very close to the previous causing snap turns
		mTangent2.rotateConstraints.endBehavior = CompletionBehavior.ADJUST_VELOCITY;
		
		//Compute init linear segment to get duration and heading at transition
		mInitHeading = start.yaw;
		double initRate = start.yawRate;
		if (!MotionUtil.arePointsCoincident(start,mTangent1)) {
			LinearPathSegment firstSeg = new LinearPathSegment(start,mTangent1);
			mInitHeading = firstSeg.getFinalState().yaw;
			initRate = firstSeg.getFinalState().yawRate;
			startTime += firstSeg.getDuration();
			mTangent1.heading = mInitHeading;
			mTangent1.magHeadingRate = initRate;
		}
		
		//=== Find perpendicular lines ===
		double m1 = -dxP2A / dyP2A;
		double m2 = -dxP2B / dyP2B;
		double b1 = T1.getY() - m1 * T1.getX();
		double b2 = T2.getY() - m2 * T2.getX();
		//Check for vertical lines
		//TODO: need to check for parallel lines too? Could probably warn for that
		//  at a higher level
		if (Math.abs(dyP2A) < EPSILON) {
			mPivot = new Point2D.Double(T1.getX(),m2*T1.getX() + b2);
		} else if (Math.abs(dyP2B) < EPSILON) {
			mPivot = new Point2D.Double(T2.getX(),m1*T2.getX() + b1);
		} else {
			mPivot = new Point2D.Double((b2-b1)/(m1-m2), (m1*b2-m2*b1)/(m1-m2));
		}

		//=== Determine direction of rotation ===

		//Initial radius vector is along pivot to T1
		double rx = T1.getX() - mPivot.getX();
		double ry = T1.getY() - mPivot.getY();
		// negative R x V = negative rotation
		double rCrossV = rx*vyUnit - ry*vxUnit;
		if (rCrossV < 0) isRotationPositive = false;

		//=== Compute the initial angle from pivot to starting point ===
		mInitialVelocityHeading = Math.atan2(vyUnit, vxUnit);
		mInitRadialAngle = Math.atan2(ry, rx);

		//=== Compute motion profile along the arc ===
		//Start with the same velocity we finished the last segment with
		double vInit = mTangent1.magVelocity;
		//Get segment constraints
		//		double a = t2Constraints.getMaxAbsAccel();
		//		double vmax = t2Constraints.getMaxAbsVelocity();
		//		mMotionProfile = new TrapezoidProfile(getLength(),vInit,vmax, a,vf2);
		MotionProfileGoal goal = new MotionProfileGoal(getLength(),mTangent2.magVelocity,mTangent2.driveConstraints.endBehavior);
		TrajectoryPoint init = new TrajectoryPoint(startTime,0,vInit,0);  //NOTE: init accel doesn't matter for Trapezoid
		mDriveProfile = TrapezoidProfileGenerator.generateProfile(mTangent2.driveConstraints.maxAbsVelocity,mTangent2.driveConstraints.maxAbsAccel,goal,init);
		
		//=== Turning ===//
		//If heading is NaN, just make the robot turn to align with the velocity vector (skid steering)
		if (java.lang.Double.isFinite(mTangent2.heading) && mTangent2.rotateConstraints != null) {
			//Extract constraints
			TrajConstraints constraints = mTangent2.rotateConstraints.clone();
			//1D Goal is relative length along the segment
			double dTheta = MotionUtil.getContinuousError(mTangent2.heading - mInitHeading, 360d);
			goal = new MotionProfileGoal(dTheta,mTangent2.magHeadingRate,constraints.endBehavior);
			//Init pos is relative to the segment
			init = new TrajectoryPoint(startTime,0,initRate,0); //NOTE: init accel doesn't matter for Trapezoid
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

	public Point2D getRobotPosition(double t) {
		//Angle of radial vector from pivot to current position on the curve
		double angle = mInitRadialAngle + getIncrementalAngle(t);
		return new Point2D.Double(mPivot.getX() + mRadius * Math.cos(angle),
				mPivot.getY() + mRadius * Math.sin(angle));
	}

	@Override
	public MotionProfile getDriveProfile() {
		return mDriveProfile;
	}

	@Override
	public Waypoint getEndPoint() {
		return mTangent2;
	}

	@Override
	public Waypoint getStartPoint() {
		return mTangent1;
	}

	@Override
	public double getVelocityHeading(double t) {
		//initial angle plus incremental
		return Math.toDegrees(mInitialVelocityHeading + getIncrementalAngle(t));
	}

	/**
	 * 
	 * @param t  abs time on path
	 * @return   angle in radians
	 */
	private double getIncrementalAngle(double t) {
		//theta = s/r
		double dTheta = getDriveTrajPoint(t).pos() / mRadius;
		if (isRotationPositive) return dTheta;
		else return -dTheta;
	}

	@Override
	public double getLength() {
		//theta is the angle covered at the pivot point, since the 
		//end points are tangent, theta is 180deg - the included angle of the waypoints
		double theta = Math.PI - mInternalAngle;
		//s = r * theta
		return mRadius * theta;
	}

	/**
	 * Determine if point is nearly on the line
	 * @param point
	 * @param line
	 * @return
	 */
	private boolean isPntOnLine(Point2D point, Line2D line) {
		Point2D.Double start = (Double) line.getP1();
		Point2D end = line.getP2();
		double err = start.distance(point) + end.distance(point) - start.distance(end);
		return Math.abs(err) < EPSILON;
	}

	@Override
	public double getRobotHeading(double t) {
		if (mRotateProfile == null) {
			//If not profiling heading, return velocity heading
			return getVelocityHeading(t);
		}
		TrajectoryPoint tp = getRotateTrajPoint(t);
		return mInitHeading + tp.pos;
	}

	@Override
	public double getRobotHeadingRate(double t) {
		TrajectoryPoint tp;
		if (mRotateProfile == null) {
			//If not profiling heading, return rate or turn
			tp = getDriveTrajPoint(t);
			double thetaDot = Math.toDegrees(tp.vel / mRadius);
			if (isRotationPositive) return thetaDot;
			else return -thetaDot;
		}
		tp = getRotateTrajPoint(t);
		return tp.vel;
	}

	@Override
	public MotionProfile getRotateProfile() {
		return mRotateProfile;
	}

}
