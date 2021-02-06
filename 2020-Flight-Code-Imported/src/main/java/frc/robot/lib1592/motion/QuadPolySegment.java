package frc.robot.lib1592.motion;

import java.awt.geom.Point2D;

public class QuadPolySegment implements PathSegment {
	private RobotState mStart;
	private Waypoint mEnd;
	private MotionProfile mDriveProfile;
	private MotionProfile mRotateProfile = null;
	//r poly is the ending
	private double[] mX_rPoly;
	private double[] mY_rPoly;
	//f poly is the starting
	private double[] mX_fPoly;
	private double[] mY_fPoly;
	private double mUstart;
	private double mUend;
	//Velocity along the path
	private double vInit;
	//relative length increments along the path
	//TODO: do I really want to save dl? Can compute from mLength
	private double[] mDeltaDist;
	//length along the segment
	private double[] mCumDist;
	//straight-line distance to integrate over
	private static final double du = 0.25; //ft
	//1D LUT for l = f(u)
	private InterpolatingDouble1D ULMap;
	private InterpolatingDouble1D UdLMap;
	

	QuadPolySegment(RobotState start, Waypoint end, double uStart, double uEnd,
			double[] x_fPoly, double[] y_fPoly, double[] x_rPoly, double[] y_rPoly) {
		//=== Capture Inputs ===//
		mStart = start;
		mEnd = end;
		mX_rPoly = x_rPoly;
		mX_fPoly = x_fPoly;
		mY_rPoly = y_rPoly;
		mY_fPoly = y_fPoly;
		mUstart = uStart;
		mUend = uEnd;
		
		//=== Numerically integrate to find length along the path vs u ===//
		//number of segments of u
		int n = (int) Math.ceil((mUend - mUstart) / du) + 1;
		mDeltaDist = new double[n];
		mCumDist = new double[n];
		double[] u = new double[n];
		u[0] = mUstart;
		//TODO: ensure last segment of u = mUend
		for (int i = 1; i < n; i++) {
			u[i] = mUstart + i*du;
			mDeltaDist[i] = Math.sqrt(dx_du(u[i])*dx_du(u[i])+dy_du(u[i])*dy_du(u[i]))*du;
			mCumDist[i] = mCumDist[i-1] + mDeltaDist[i];
		}
		//TODO: should mDeltaDist be initialized with something different
		mDeltaDist[0] = mDeltaDist[1];
		
		ULMap = new InterpolatingDouble1D(u, mCumDist);
		UdLMap = new InterpolatingDouble1D(u,mDeltaDist);
		
		
		//=== Drive Motion Profile ===//
		//TODO: warn if velocity is significantly off the path?
		double angleInit = Math.atan2(dy_du(mUstart), dx_du(mUstart));
		vInit = start.xVelocity*Math.cos(angleInit) + start.yVelocity*Math.sin(angleInit);
		MotionProfileGoal goal = new MotionProfileGoal(getLength(),mEnd.magVelocity,mEnd.driveConstraints.endBehavior);
		//Init pos and time start at zero because they are relative to the segment
		TrajectoryPoint init = new TrajectoryPoint(start.time,0,vInit,0); //NOTE: init accel doesn't matter for Trapezoid
		mDriveProfile = TrapezoidProfileGenerator.generateProfile(mEnd.driveConstraints.maxAbsVelocity,mEnd.driveConstraints.maxAbsAccel,goal,init);

		//=== Rotation Motion Profile ===//
		//If heading is NaN, just make the robot turn to align with the velocity vector (skid steering)
		if (Double.isFinite(mEnd.heading) && mEnd.rotateConstraints != null) {
			//Extract constraints
			TrajConstraints constraints = mEnd.rotateConstraints.clone();
			//1D Goal is relative length along the segment
			double dTheta = MotionUtil.getContinuousError(mEnd.heading - mStart.yaw, 360d);
			goal = new MotionProfileGoal(dTheta,mEnd.magHeadingRate,constraints.endBehavior);
			//Init pos and time start at zero because they are relative to the segment
			//TODO: do we need to do this profile relative?  Can we just eliminate this?  Right now, it makes the interface 
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
	
	
	private double getU(double t) {
		if (t <= getStartTime()) return mUstart;
		else if (t >= getEndTime()) return mUend;
		else {
			TrajectoryPoint tp_l = getDriveTrajPoint(t);
			return ULMap.invInterp(tp_l.pos());
		}
	}
	
	/**
	 * Declare this is the start point if the initial cumulative
	 * distance is 0
	 * 
	 * @return true if the start of path
	 */
	private boolean isPathStartPoint() {
		return mUstart == 0;
	}
	
	/**
	 * Declare this is the end point if the starting poly coeffs are zeros
	 * 
	 * @return true if the end of path
	 */
	private boolean isPathEndPoint() {
		double sum = 0;
		//Use the fact that we know polys have 3 coeffs
		for (int i=0; i < 3; i++) {
			sum += mX_rPoly[i];
			sum += mY_rPoly[i];
		}
		return Math.abs(sum) <= MotionUtil.EPSILON;
	}
	
	/**
	 * X position along the path vs u
	 * @param u  straight line distance relative to the path
	 * @return  x position
	 */
	private double x_u(double u) {
		return mu_r(u) * (mX_rPoly[0]*u*u + mX_rPoly[1]*u + mX_rPoly[2]) + 
				mu_f(u) * (mX_fPoly[0]*u*u + mX_fPoly[1]*u + mX_fPoly[2]);
	}
	
	/**
	 * Y position along the path vs u
	 * @param u  straight line distance relative to the path
	 * @return  y position
	 */
	private double y_u(double u) {
		return mu_r(u) * (mY_rPoly[0]*u*u + mY_rPoly[1]*u + mY_rPoly[2]) + 
			   mu_f(u) * (mY_fPoly[0]*u*u + mY_fPoly[1]*u + mY_fPoly[2]);
	}
	
	/**
	 * First derivative of x position w.r.t u
	 * 
	 * @param u  straight-line distance relative to the path
	 * @return  dx/du
	 */
	private double dx_du(double u) {
		return mu_r(u) * (2*mX_rPoly[0]*u + mX_rPoly[1]) + 
			   mu_f(u) * (2*mX_fPoly[0]*u + mX_fPoly[1]);
	}
	
	/**
	 * First derivative of y position w.r.t u
	 * 
	 * @param u  straight-line distance relative to the path
	 * @return  dy/du
	 */
	private double dy_du(double u) {
		return mu_r(u) * (2*mY_rPoly[0]*u + mY_rPoly[1]) + 
			   mu_f(u) * (2*mY_fPoly[0]*u + mY_fPoly[1]);
	}
	
	/**
	 * Second derivative of x position w.r.t u
	 * 
	 * @param u  straight-line distance relative to the path
	 * @return  d2x/du2
	 */
	private double d2x_du2(double u) {
		return mu_r(u) * 2*mX_rPoly[0] + mu_f(u) * 2*mX_fPoly[0];
	}
	
	/**
	 * Second derivative of y position w.r.t u
	 * 
	 * @param u  straight-line distance relative to the path
	 * @return  d2y/du2
	 */
	private double d2y_du2(double u) {
		return mu_r(u) * 2*mY_rPoly[0] + mu_f(u) * 2*mY_fPoly[0];
	}
	
	/**
	 * Rising fuzzy logic membership function
	 * 
	 * @param u  straight line distance along the path
	 * @return  mu_r
	 */
	private double mu_r(double u) {
		if (isPathStartPoint()) return 1;
		else if (isPathEndPoint()) return 0;
		else return (u - mUstart) / (mUend - mUstart);
	}
	
	/**
	 * Signed curvature w.r.t u.  I think this is equivalent to 
	 * 1/p where p is the radius of curvature in length units.
	 * 
	 * @param u  straight line distance along the path
	 * @return  curvature
	 */
	double getCurvature(double u) {
		double num = dx_du(u)*d2y_du2(u) - dy_du(u)*d2x_du2(u);
		double den = Math.pow(dx_du(u)*dx_du(u)+dy_du(u)*dy_du(u), 1.5);
		return num/den;
	}
	
	/**
	 * Falling fuzzy logic membership function
	 * 
	 * @param u  straight line distance along the path
	 * @return  mu_f
	 */
	private double mu_f(double u) {
		if (isPathStartPoint()) return 0;
		else if (isPathEndPoint()) return 1;
		else return (mUend - u) / (mUend - mUstart);
	}

	@Override
	public MotionProfile getDriveProfile() {
		return mDriveProfile;
	}

	@Override
	public MotionProfile getRotateProfile() {
		return mRotateProfile;
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
	public Point2D getRobotVelocity(double t) {
		//Get du/dl
		double u = getU(t);
		double dl = UdLMap.interp(u);
		double du_dl = du / dl;
		//Get dl/dt
		TrajectoryPoint tp_l = getDriveTrajPoint(t);
		double dl_dt = tp_l.vel();
		//Compute velocity
		double dx_dt = dx_du(u)*du_dl*dl_dt;
		double dy_dt = dy_du(u)*du_dl*dl_dt;
		return new Point2D.Double(dx_dt,dy_dt);
	}

	@Override
	public double getVelocityHeading(double t) {
		double u = getU(t);
		return Math.toDegrees(Math.atan2(dy_du(u), dx_du(u)));
	}

	@Override
	public Point2D getRobotPosition(double t) {
		double u = getU(t);
		return new Point2D.Double(x_u(u),y_u(u));
	}

	@Override
	public double getRobotHeading(double t) {
		if (mRotateProfile == null) {
			//If not profiling heading, return velocity heading
			return getVelocityHeading(t);
		}
		TrajectoryPoint tp = getRotateTrajPoint(t);
		return mStart.yaw + tp.pos;
	}

	@Override
	public double getRobotHeadingRate(double t) {
		TrajectoryPoint tp_l;
		//TODO: check this calc
		if (mRotateProfile == null) {
			//If not profiling heading, return rate or turn
			tp_l = getDriveTrajPoint(t);
			double u = ULMap.invInterp(tp_l.pos());
			double curvature = getCurvature(u);
			return Math.toDegrees(tp_l.vel * curvature);
		}
		return getRotateTrajPoint(t).vel;
	}

	@Override
	public double getLength() {
		return ULMap.mY[ULMap.length()-1];
	}

}
