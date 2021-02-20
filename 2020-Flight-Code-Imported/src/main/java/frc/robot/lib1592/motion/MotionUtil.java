package frc.robot.lib1592.motion;

import java.awt.geom.Point2D;

public class MotionUtil {
	
	static final double EPSILON = 1e-6;
	
	static final double SAME_TIME = 0.02; //s

	/**
	 * Find the internal angle in the triangle formed by points 1, 2 (vertex), and 3
	 * using the law of cosines: c^2 = a^2 + b^2 -2ac*cos(theta)
	 * 
	 * @param pnt1
	 * @param pnt2
	 * @param pnt3
	 * @return theta
	 */
	public static double getInternalAngle(Point2D pnt1, Point2D pnt2, Point2D pnt3) {
		//a is line b/n point 1 and 2
		double dx = pnt2.getX()-pnt1.getX();
		double dy = pnt2.getY()-pnt1.getY();
		double a = Math.sqrt(dx*dx+dy*dy);
		//b is line b/n point 2 and 3
		dx = pnt2.getX()-pnt3.getX();
		dy = pnt2.getY()-pnt3.getY();
		double b = Math.sqrt(dx*dx+dy*dy);
		//c is line b/n point 1 and 3
		dx = pnt1.getX()-pnt3.getX();
		dy = pnt1.getY()-pnt3.getY();
		double c = Math.sqrt(dx*dx+dy*dy);
		
		return Math.acos((a*a + b*b - c*c) / (2*a*b));
	}
	
	/**
	 * Are points nearly identical
	 * 
	 * @param p1
	 * @param p2
	 * @return
	 */
	public static boolean arePointsCoincident(Point2D p1, Point2D p2) {
		return p1.distance(p2) < EPSILON;
	} 
	
	/**
	 * Are 2 values equal within the tolerance, eps
	 * @param a
	 * @param b
	 * @param eps
	 * @return
	 */
	public static boolean epsEquals(double a, double b, double eps) {
        return (a - eps <= b) && (a + eps >= b);
    }
	
	  /**
	   * Wraps error around for continuous inputs.
	   *
	   * @param error    difference in continuous inputs
	   * @return error   wrapped error
	   */
	public static double getContinuousError(double error, double inputRange) {
		error %= inputRange;
		if (Math.abs(error) > inputRange / 2) {
			if (error > 0) {
				error -= inputRange;
			} else {
				error += inputRange;
			}
		}
		return error;
	}
}
