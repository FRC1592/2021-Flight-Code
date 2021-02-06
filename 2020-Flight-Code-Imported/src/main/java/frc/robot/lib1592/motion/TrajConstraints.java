package frc.robot.lib1592.motion;

import frc.robot.lib1592.motion.MotionProfileGoal.CompletionBehavior;

public class TrajConstraints {
	
	protected double maxAbsVelocity;
	protected double maxAbsAccel;
	protected CompletionBehavior endBehavior = CompletionBehavior.OVERSHOOT;

	/**
	 * Max absolute acceleration and velocity
	 * 
	 * @param v
	 * @param a
	 */
	public TrajConstraints(double v, double a) {
		maxAbsVelocity = Math.abs(v);
		maxAbsAccel = Math.abs(a);
	}
	
	public TrajConstraints(double v, double a, CompletionBehavior c) {
		maxAbsVelocity = Math.abs(v);
		maxAbsAccel = Math.abs(a);
		endBehavior = c;
	}
	
	public double getMaxAbsVelocity() {
		return maxAbsVelocity;
	}
	
	public double getMaxAbsAccel() {
		return maxAbsAccel;
	}
	
	public TrajConstraints clone() {
		return new TrajConstraints(maxAbsVelocity,maxAbsAccel,endBehavior);
	}
	
	public String toString() {
		return new String("max v: " + maxAbsVelocity + ", accel: " + maxAbsAccel + 
				", completion behavior: " + endBehavior.name());
	}
}
