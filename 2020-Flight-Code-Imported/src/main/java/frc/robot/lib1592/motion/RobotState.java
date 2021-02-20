package frc.robot.lib1592.motion;

import java.awt.geom.Point2D;
import java.text.DecimalFormat;
import java.text.NumberFormat;

public class RobotState extends Point2D{
	protected double time = java.lang.Double.NaN;
	protected double xPosition = 0;
	protected double yPosition = 0;
	protected double xVelocity = 0;
	protected double yVelocity = 0;
	protected double yaw = 0;
	protected double yawRate = 0;
	
	/**
	 * Construct default state with time = NaN
	 */
	public RobotState() {}
	
	/**
	 * Construct position, velocity, and attitude state
	 * 
	 * @param x			inertial x position
	 * @param y			inertial y position
	 * @param xdot		inertial x velocity
	 * @param ydot		inertial y velocity
	 * @param psi		angle from body to inertial frame [deg]
	 * @param psidot 	rotation rate  [deg/s]
	 * @param t			time of state validity
	 */
	public RobotState(double x, double y, double xdot, double ydot, double psi, double psidot, double t) {
		xPosition = x;
		yPosition = y;
		xVelocity = xdot;
		yVelocity = ydot;
		yaw = psi;
		yawRate = psidot;
		time = t;
	}
	
	/**
	 * Construct position, velocity, and attitude state
	 * 
	 * @param pos		ienrtial position vector
	 * @param vel		inertial velocity vector
	 * @param psi		angle from body to inertial frame [deg]
	 * @param psidot 	rotation rate [deg/s]
	 * @param t			time of state validity
	 */
	public RobotState(Point2D pos, Point2D vel, double psi, double psidot, double t) {
		this(pos.getX(),pos.getY(),vel.getX(),vel.getY(),psi,psidot, t);
		}

	@Override
	public String toString() {
		NumberFormat formatter = new DecimalFormat("#0.000");
		return  formatter.format(time) + ", " +
				formatter.format(xPosition) + ", " + formatter.format(yPosition) + ", " + 
				formatter.format(xVelocity) + ", " + formatter.format(yVelocity)+ ", " +
				formatter.format(yaw) 		+ ", " + formatter.format(yawRate);
	}
	
	@Override
	public double getX() {
		return xPosition;
	}

	@Override
	public double getY() {
		return yPosition;
	}
	
	public double getTime() {
		return time;
	}
	
	public Point2D getPosition() {
		return new Point2D.Double(xPosition,yPosition);
	}
	
	public Point2D getVelocity() {
		return new Point2D.Double(xVelocity,yVelocity);
	}
	
	/**
	 * @return  yaw angle from body to inertial in degrees
	 */
	public double getYaw() {
		return yaw;
	}
	
	/**
	 * @return  body rotation rate in deg/s
	 */
	public double getYawRate() { 
		return yawRate;
	}

	@Override
	public void setLocation(double x, double y) {
		xPosition = x;
		yPosition = y;
		
	}
	
}