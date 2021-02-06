package frc.robot.lib1592.utils;

public class RateLimiter {

	//The value from the last cycle
	protected double lastValue = 0d;
	//The max change towards a non-zero target
	protected double maxChange;
	//The maximum change when approaching zero
	protected double maxChange2Zero;
	//Threshold below which the target is considered to be zero
	protected double zeroThreshold = 0.001;
//	private double dt = 1d;
	
	/**
	 * Constructor
	 * Set limits on rate-of-change towards any target
	 * @param rateLimit
	 */
	public RateLimiter(double rateLimit) {
		maxChange = rateLimit;
		maxChange2Zero = rateLimit;
	}
	
	/**
	 * Constructor
	 * Set individual limits on rate-of-change towards zero and non-zero targets
	 * @param rateLimit
	 * @param stoppingRateLimit
	 */
	public RateLimiter(double rateLimit, double stoppingRateLimit) {
		maxChange = rateLimit;
		setStoppingRateLimit(stoppingRateLimit);
	}
	
	/**
	 * Limit the rate of change of the input parameter
	 * @param in
	 * @return in_rateLimited
	 */
	public double limitRate(double in) {
		double out;
		double lim;
		if (Math.abs(in) > zeroThreshold) {
			//Limit rate towards target
			lim = maxChange;
		} else {
			//Decay to zero
			lim = maxChange2Zero;
		}
		out = Discontinuities.limit(in,lastValue-lim,lastValue+lim);
		lastValue = out;
		return out;
	}
	
	/**
	 * Update previous value for evaluating rate of change.
	 * @param lastVal
	 */
	public void setLastValue(double lastVal) {
		lastValue = lastVal;
	}
	
	/**
	 * Set the threshold below which the target value is considered zero
	 * @param thresh
	 */
	public void setZeroThreshold(double thresh) {
		//Needs to be positive value
		if (thresh < 0) {thresh = -thresh;}
		zeroThreshold = thresh;
	}
	
	/**
	 * Sets the limit on the rate of change when returning to zero
	 * 
	 * @param stoppingRateLimit
	 */
	public void setStoppingRateLimit(double stoppingRateLimit) {
		//Needs to be a positive value
		if (stoppingRateLimit < 0) {
			stoppingRateLimit = -stoppingRateLimit;
		}
		
		maxChange2Zero = stoppingRateLimit;
	}
	
	/**
	 * Sets the limit on the rate of change when approaching non-zero targets
	 * @param rateLimit
	 */
	public void setRateLimit(double rateLimit) {
		if (rateLimit < 0) {
			rateLimit = -rateLimit;
		}
		maxChange = rateLimit;
	}
	
	
}
