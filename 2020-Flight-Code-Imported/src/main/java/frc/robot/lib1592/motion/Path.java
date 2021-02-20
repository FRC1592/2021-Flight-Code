package frc.robot.lib1592.motion;

import java.awt.geom.Point2D;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;

import frc.robot.lib1592.motion.MotionProfileGoal.CompletionBehavior;
import frc.robot.lib1592.motion.CurvedPathSegment;

public class Path {

	protected List<PathSegment> mSegments;

	Path(List<PathSegment> segments) {
		mSegments = segments;
	}

	public static Path of(List<Waypoint> waypoints, RobotState start) {
		return generatePath(waypoints, start);
	}

	/**
	 * Get the total amount of time to complete the path
	 * @param start  initial position and velocity
	 * @return time  path duration
	 */
	public double getDuration() {
		double cumTime = 0;
		for (PathSegment seg : mSegments) {
			cumTime += seg.getDuration();
		}
		return cumTime;
	}
	
	/**
	 * Get the final waypoint/target position on the path
	 * 
	 * @return  x/y position of the path terminal point
	 */
	public Point2D getFinalPosition() {
		int n = mSegments.size();
		return mSegments.get(n-1).getEndPoint();
	}
	
	/**
	 * Get the final waypoint/target yaw position on the path
	 * 
	 * @return  yaw angle command [deg] at the terminal path point
	 */
	public double getFinalYaw() {
		int n = mSegments.size();
		return mSegments.get(n-1).getEndPoint().heading;
	}

	
	/**
	 * Get the initial time on the path
	 * 
	 * @return  path relative time in seconds
	 */
	public double startTime() {
		if (mSegments.isEmpty()) {
			return Double.NaN;
		} else {
			return mSegments.get(0).getStartTime();
		}
	}
	
	public double endTime() {
		if (mSegments.isEmpty()) {
			return Double.NaN;
		} else {
			return startTime() + getDuration();
		}
	}

	/**
	 * Publish the path to ascii files - one containing the 1D motion profile and the other containing the 
	 * position and velocity data along the path
	 * 
	 * @param start   		robot initial state
	 * @param t				time vector to interpolate along the path
	 * @param mpFile		1D motion profile file
	 * @param pathFile		path data file
	 * @throws IOException
	 */
	public void writePathtoFile(double [] t, File mpFile, File pathFile) throws IOException {

		PrintWriter mpWriter = new PrintWriter(new FileWriter(mpFile,false));
		PrintWriter pathWriter = new PrintWriter(new FileWriter(pathFile,false));

		double pStart = 0;
		double rStart = mSegments.get(0).getStartPoint().heading;
		int iSegment = 0;
		for (int i = 0; i < t.length; i++) {
			double tOnSegment = t[i]; 
			if (!mSegments.get(iSegment).containsTime(t[i]) && iSegment < mSegments.size()-1) {
				//Motion distances are relatvie to the start point, so we have to keep track
				pStart += mSegments.get(iSegment).getLength();
				rStart = mSegments.get(iSegment).getFinalState().yaw;
				iSegment++;
			}
			TrajectoryPoint pnt = mSegments.get(iSegment).getDriveTrajPoint(tOnSegment);
			TrajectoryPoint rPnt = mSegments.get(iSegment).getRotateTrajPoint(tOnSegment);
			RobotState state = mSegments.get(iSegment).getRobotState(tOnSegment);
			TrajectoryPoint pOut = new TrajectoryPoint(pnt.t,pnt.pos+pStart,pnt.vel,pnt.acc);
			TrajectoryPoint rOut = new TrajectoryPoint(rPnt.t,rPnt.pos+rStart,rPnt.vel,rPnt.acc);
			mpWriter.println(pOut.toString() + ", " + rOut.toString() + ", " + iSegment);
			pathWriter.println(tOnSegment + ", " + state.xPosition + ", " + state.yPosition + ", " + 
							state.xVelocity + ", " + state.yVelocity + ", " + state.yaw + ", " + state.yawRate);
//			pathWriter.println(state.toString());
		}

		mpWriter.close();
		pathWriter.close();

	}
	
	public void checkPathValidity() {
		boolean isPathValid = true;
		for (int i = 0; i < mSegments.size() - 1; i++) {
			double endTime = mSegments.get(i).getEndTime();
			double startTime = mSegments.get(i+1).getStartTime();
			if (!MotionUtil.epsEquals(endTime,startTime,MotionUtil.EPSILON)) {
				isPathValid = false;
				System.err.println("Segment start and end times don't equal");
				System.out.println("segment" + i + " end: " + endTime);
				System.out.println("segment" + (i+1) + " start: " + startTime);
			}
		}
		if (isPathValid) System.out.println("Path is valid");;
	}
	
	/**
	 * Get the robot state on the path at a given time
	 * @param t  time on the path
	 * @return  full robot state
	 */
	public RobotState getRobotState(double t) {
		 if (t < startTime()) {
	            t = startTime();
	        } else if (t > endTime()) {
	            t = endTime();
	        }
	        for (PathSegment s : mSegments) {
	            if (s.containsTime(t)) {
	                return s.getRobotState(t);
	            }
	        }
	        //NOTE: shouldn't get here
	        System.err.println("Warning in Path.getRobotState: Overran interpolation");
	        return mSegments.get(mSegments.size()-1).getRobotState(endTime());
	}

	public static Path generatePath(List<Waypoint> waypoints, RobotState start) {
		//Check the last waypoint and ensure it stops
		if (waypoints.get(waypoints.size()-1).magVelocity > 0) {
			System.err.println("Final Waypoint must finish with zero velocity.");
			waypoints.get(waypoints.size()-1).magVelocity = 0;
		}
		if (waypoints.get(waypoints.size()-1).magHeadingRate > 0) {
			System.err.println("Final Waypoint must finish with zero rate.");
			waypoints.get(waypoints.size()-1).magHeadingRate = 0;
		}
		
		//Check the last waypoint and ensure the finish behavior is consistent
		if (waypoints.get(waypoints.size()-1).driveConstraints.endBehavior.equals(CompletionBehavior.ADJUST_VELOCITY)) {
			System.err.println("Final waypoint can't use ADJUST_VELOCITY.  Setting to ADJUST_ACCEL");
			waypoints.get(waypoints.size()-1).driveConstraints.endBehavior = CompletionBehavior.ADJUST_ACCEL;
		}
		//Initialize
		List<PathSegment> segments = new ArrayList<PathSegment>();
		RobotState prevState = start;
		//Loop through waypoints to create the segments
		for (int i = 0; i < waypoints.size()-1; i++) {
			//Identify sharp corners in the path
			double angle = MotionUtil.getInternalAngle(prevState, waypoints.get(i), waypoints.get(i+1));
			//If too sharp, insert an arc to transition.  Unless coming to a full stop
			if (waypoints.get(i).magVelocity > 0 && angle > 0 && angle < waypoints.get(i).minAngle) {
				//Created a curved segment
				CurvedPathSegment seg = new CurvedPathSegment(prevState, waypoints.get(i), waypoints.get(i+1));
				//Create the straight segment before the curve if we didn't curve the whole thing
				if (!MotionUtil.arePointsCoincident(prevState,seg.getStartPoint())) {
					segments.add(new LinearPathSegment(prevState, seg.getStartPoint()));
				}
				//Replace the old waypoint with the end of the curved segment
				segments.add(seg);
				//Capture state at the waypoint for the next iter
				prevState = seg.getFinalState();
				//Have we already gone all the way to the next waypoint? If so, remove it (except the last)
				if (i != waypoints.size()-2 && MotionUtil.arePointsCoincident(prevState,waypoints.get(i+1))) {
					waypoints.remove(i+1);
				}
			} else {
				LinearPathSegment seg = new LinearPathSegment(prevState,waypoints.get(i));
				segments.add(seg);
				//Capture state at the waypoint for the next iter
				prevState = seg.getFinalState();
			}
		}
		//Final segment will be straight
		int iLast = waypoints.size()-1;
		//Check to make sure the last smoothing operation didn't eliminate the point
		if (!MotionUtil.arePointsCoincident(prevState,waypoints.get(iLast))) {
			segments.add(new LinearPathSegment(prevState, waypoints.get(iLast)));
		}

		Path path = new Path(segments);
		path.checkPathValidity();
		return path;
	}
	
	
	public static Path generatePolyPath(List<Waypoint> waypoints, RobotState start) {
		//m is the number of points = waypoints + start
		int m = waypoints.size() + 1;

		//Special case:  If only start and 1 waypoint, use generatePath to produce 1 straight segment
		if (m == 2) return generatePath(waypoints, start);

		//Check the last waypoint and ensure it stops
		if (waypoints.get(waypoints.size()-1).magVelocity > 0) {
			System.err.println("Final Waypoint must finish with zero velocity.");
			waypoints.get(waypoints.size()-1).magVelocity = 0;
		}
		if (waypoints.get(waypoints.size()-1).magHeadingRate > 0) {
			System.err.println("Final Waypoint must finish with zero rate.");
			waypoints.get(waypoints.size()-1).magHeadingRate = 0;
		}
			
		//Check the last waypoint and ensure the finish behavior is consistent
		if (waypoints.get(waypoints.size()-1).driveConstraints.endBehavior.equals(CompletionBehavior.ADJUST_VELOCITY)) {
			System.err.println("Final waypoint can't use ADJUST_VELOCITY.  Setting to ADJUST_ACCEL");
			waypoints.get(waypoints.size()-1).driveConstraints.endBehavior = CompletionBehavior.ADJUST_ACCEL;
		}
		//Initialize
		List<PathSegment> segments = new ArrayList<PathSegment>();
		
		//u contains the cumulative linear distances between waypoints
		double[] u = new double[m];
		//Poly coefficients (initialized filled with zeros)
		double abc_x[][] = new double[m][3];
		double abc_y[][] = new double[m][3];
		
		//=== Populate linear distances between waypoints ===//
		u[0] = 0;
		u[1] = start.distance(waypoints.get(0));
		for (int n = 2; n < m; n++) {
			u[n] = u[n-1] + waypoints.get(n-2).distance(waypoints.get(n-1));
		}
		
		//=== Compute the poly coefficients ===//
		//First point needs to use start state
		double[] tmp = calcPolyCoeffs(u[0],u[1],u[2],
				start.getX(),waypoints.get(0).getX(),waypoints.get(1).getX());
		abc_x[1][0] = tmp[0];
		abc_x[1][1] = tmp[1];
		abc_x[1][2] = tmp[2];
		tmp = calcPolyCoeffs(u[0],u[1],u[2],
				start.getY(),waypoints.get(0).getY(),waypoints.get(1).getY());
		abc_y[1][0] = tmp[0];
		abc_y[1][1] = tmp[1];
		abc_y[1][2] = tmp[2];
		//Loop over the rest of the waypoints
		for (int n = 2; n < m-1; n++) {
			tmp = calcPolyCoeffs(u[n-1],u[n],u[n+1],
					waypoints.get(n-2).getX(),waypoints.get(n-1).getX(),waypoints.get(n).getX());
			abc_x[n][0] = tmp[0];
			abc_x[n][1] = tmp[1];
			abc_x[n][2] = tmp[2];
			
			tmp = calcPolyCoeffs(u[n-1],u[n],u[n+1],
					waypoints.get(n-2).getY(),waypoints.get(n-1).getY(),waypoints.get(n).getY());
			abc_y[n][0] = tmp[0];
			abc_y[n][1] = tmp[1];
			abc_y[n][2] = tmp[2];
		}
		
		//=== Create Path Segments ===//
		RobotState prevState = start;
		for (int n = 0; n < m-1; n++) {
			PathSegment seg = new QuadPolySegment(prevState,waypoints.get(n),u[n],u[n+1],
					abc_x[n],abc_y[n],abc_x[n+1],abc_y[n+1]);
			prevState = seg.getFinalState();
			segments.add(seg);
		}
		
		return new Path(segments);
	}

	/**
	 * Computes the coefficients [a,b,c] from the equation below (derived in Matlab) 
	 *	[a]   [ u1^2, u1, 1]^-1   [x1]
	 *  [b] = [ u2^2, u2, 1]    x [x2]
	 *  [c]   [ u3^2, u3, 1]      [x3]
	 *  
	 * @return {a,b,c}
	 */
	private static double[] calcPolyCoeffs(double u1, double u2, double u3, double x1, double x2, double x3) {
		double a = x3/(u1*u2 - u1*u3 - u2*u3 + u3*u3) - x1/(u1*u2 + u1*u3 - u2*u3 - u1*u1) - x2/(u1*u2 - u1*u3 + u2*u3 - u2*u2);
		double b = (x1*(u2 + u3))/(u1*u2 + u1*u3 - u2*u3 - u1*u1) - (x3*(u1 + u2))/(u1*u2 - u1*u3 - u2*u3 + u3*u3) + (x2*(u1 + u3))/(u1*u2 - u1*u3 + u2*u3 - u2*u2);
		double c = (u1*u2*x3)/(u1*u2 - u1*u3 - u2*u3 + u3*u3) - (u2*u3*x1)/(u1*u2 + u1*u3 - u2*u3 - u1*u1) - (u1*u3*x2)/(u1*u2 - u1*u3 + u2*u3 - u2*u2);
		double[] abc = {a,b,c};
		return abc;
	}
	
}
