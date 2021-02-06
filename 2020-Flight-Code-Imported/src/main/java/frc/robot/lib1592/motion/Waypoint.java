/**
 * 
 */
package frc.robot.lib1592.motion;

import java.awt.geom.Point2D;

import frc.robot.lib1592.utils.Vector2D;

/**
 * @author ddyer
 *
 */
public class Waypoint extends Point2D{
	//=== Translation Data ===//
	protected double xPosition;
	protected double yPosition;
	protected TrajConstraints driveConstraints;
	protected double magVelocity = 0;

	//=== Rotation Data ===//
	protected double heading = java.lang.Double.NaN;
	//TODO: Not part of public interface (constructor) but using to hand-off if rotation profile didn't finish
	double magHeadingRate = 0;
	protected TrajConstraints rotateConstraints = null;
	protected RotateBehavior rotateBehavior = RotateBehavior.EARLY;

	//=== Smoothing Options ===//
	//Radius of fillet if smoothing sharp path corners
	//TODO: may want to parameterize a little different.  Max miss distance? Approach radius (distance to start turning)?
	//To minimize skidding, smoothing radius as a function of velocity may be most appropriate (radialAccel = -r*omega^2)
	protected double smoothingRadius = java.lang.Double.POSITIVE_INFINITY;
	//Any larger angle is considered a straight segment
	protected double minAngle = 165d * Math.PI/180d;
	
	public class DriveGoal {
		double x;
		double y;
		TrajConstraints constraints;
		double vMag = 0;
		
		public DriveGoal(double x, double y, TrajConstraints c, double v) {
			this.x = x;
			this.y = y;
			constraints = c.clone();
			vMag = v;
		}
	}
	
	public class RotateGoal {
		double heading;
		TrajConstraints constraints;
		double vMag = 0;
		
		public RotateGoal(double h, TrajConstraints c, double v) {
			heading = h;
			constraints = c.clone();
			vMag = v;
		}
	}
	
	/**
	 * Constructor - Create waypoint using Drive and Rotate Goal Containers
	 * 
	 * @param driveGoal
	 * @param rotateGoal
	 */
	public Waypoint(DriveGoal driveGoal, RotateGoal rotateGoal) {
		this(driveGoal.x,driveGoal.y,driveGoal.constraints,driveGoal.vMag,
				rotateGoal.heading,rotateGoal.constraints);
	}
	
	/**
	 * Full Constructor - Create waypoint using Drive and Rotate Goal Containers
	 * @param driveGoal
	 * @param rotateGoal
	 * @param rSmooth
	 * @param angle
	 */
	public Waypoint(DriveGoal driveGoal, RotateGoal rotateGoal, double rSmooth, double angle) {
		this(driveGoal.x,driveGoal.y,driveGoal.constraints,driveGoal.vMag,
				rotateGoal.heading,rotateGoal.constraints,
				rSmooth,angle);
	}

	/**
	 * Minimal constructor with only position and kinematic constraints.
	 * Sets final velocity to 0
	 * 
	 * @param x				y position
	 * @param y				x position
	 * @param driveConstr 	velocity and acceleration constraints
	 */
	public Waypoint(double x, double y, TrajConstraints driveConstr) {
		this(x,y,driveConstr,0);
		
	}
	
	/**
	 * Constructor - Drive only with specification for position, velocity and constraints
	 * @param x				y position
	 * @param y				x position
	 * @param driveConstr 	velocity and acceleration constraints
	 * @param vmag 			velocity magnitude at end
	 */
	public Waypoint(double x, double y, TrajConstraints driveConstr, double vmag) {
		this(x,y,driveConstr,vmag,java.lang.Double.NaN,null);
		
	}
	
	/**
	 * Constructor - Specify full drive specification
	 * 
	 * @param x				y position
	 * @param y				x position
	 * @param driveConstr 	velocity and acceleration constraints
	 * @param vmag 			velocity magnitude at end
	 * @param rSmooth		radius of fillet if smoothing sharp path corners
	 * @param angle			any larger angle (rad) is considered a straight segment
	 */
	public Waypoint(double x, double y, TrajConstraints driveConstr , double vmag, double rSmooth, double angle) {
		this(x,y,driveConstr,vmag,java.lang.Double.NaN,null,rSmooth,angle);
		
	}
	
	/**
	 * Constructor - Drive and Rotation specification using defaults smoothing
	 * @param x				y position
	 * @param y				x position
	 * @param driveConstr 	velocity and acceleration constraints
	 * @param vmag 			velocity magnitude at end
	 * @param head			heading angle in rad
	 * @param rotConstr		velocity and acceleration constraints
	 */
	public Waypoint(double x, double y, TrajConstraints driveConstr , double vmag, double head, TrajConstraints rotConstr) {
		xPosition = x;
		yPosition = y;
		driveConstraints = driveConstr.clone();
		//Ensure 0 <= v <= maxVelocity
		magVelocity = Math.min(Math.abs(vmag),driveConstr.maxAbsVelocity);
		rotateConstraints = rotConstr;
		heading = head;
	}
	
	/**
	 * Full Constructor
	 * 
	 * @param x				y position
	 * @param y				x position
	 * @param driveConstr 	velocity and acceleration constraints
	 * @param vmag 			velocity magnitude at end
	 * @param head			heading angle in rad
	 * @param rotConstr		velocity and acceleration constraints
	 * @param rSmooth		radius of fillet if smoothing sharp path corners
	 * @param angle			any larger angle (rad) is considered a straight segment
	 */
	public Waypoint(double x, double y, TrajConstraints driveConstr , double vmag, double head, TrajConstraints rotConstr, double rSmooth, double angle) {
		this(x, y, driveConstr, vmag,head, rotConstr);
		if (rSmooth <=0) {
			System.err.println("rSmooth must be > 0.  Setting smoothingRadius to 0 so no curves happen");
			smoothingRadius = 0;
			minAngle = 0;
		} else {
			smoothingRadius = rSmooth;
		}
		if (minAngle < 0 || minAngle > 180) {
			System.err.println("Invalid minAngle = " + angle + " deg. Must be on range [0 180deg]. Using default value of " + minAngle +  " deg");
		} else
			minAngle = angle;
	}
	
	/**
	 * Get deep copy of the Waypoint
	 * 
	 * @param w  Waypoint to copy
	 * @return  deep copy of the waypoint
	 */
	public Waypoint copy() {
		Waypoint w = this;
		return new Waypoint(w.xPosition,w.yPosition,w.driveConstraints.clone(),w.magVelocity,w.heading,w.rotateConstraints.clone(),w.smoothingRadius,w.minAngle);
	}
	
	public Waypoint mirror(boolean mirrorAboutX, boolean mirrorAboutY) {
		Waypoint newWaypoint = this.copy();
		Point2D pos = newWaypoint;
		double heading = newWaypoint.heading;
		if (mirrorAboutX) {
			pos.setLocation(pos.getX(), -pos.getY());
			Vector2D dir = Vector2D.fromPolar(heading, 1);
			dir.setY(-dir.getY());
			newWaypoint.heading = dir.getAngleDeg();
		}
		if (mirrorAboutY) {
			pos.setLocation(-pos.getX(), pos.getY());
			Vector2D dir = Vector2D.fromPolar(heading, 1);
			dir.setX(-dir.getX());
			newWaypoint.heading = dir.getAngleDeg();
		}
		return newWaypoint;
	}

	@Override
	public double getX() {
		return xPosition;
	}

	@Override
	public double getY() {
		return yPosition;
	}

	@Override
	public void setLocation(double x, double y) {
		xPosition = x;
		yPosition = y;
		
	}
	
    @Override
    public String toString() {
        return "pos x: " + xPosition + ", pos y: " + yPosition + ", mag_vel: " + magVelocity + ", heading: " + heading
                + ", " + driveConstraints;
    }
	
}
