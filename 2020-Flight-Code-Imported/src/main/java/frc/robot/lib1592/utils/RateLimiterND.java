package frc.robot.lib1592.utils;

public class RateLimiterND {
	
	//The value from the last cycle
	protected double[] lastValue;
	//The max change towards a non-zero target
	protected double maxChange;
	//The maximum change when approaching zero
	protected double maxChange2Zero;
	//Threshold below which the target is considered to be zero
	protected double zeroThreshold = 0.001;

	/**
	 * Constructor
	 * Set limits on rate-of-change towards any target.
	 * @param N Length of the array to be processed
	 * @param rateLimit
	 */
	public RateLimiterND(int N, double rateLimit) {
		maxChange = rateLimit;
		lastValue = new double[N];
		//Preallocate with zeros
		for (int i=0; i < lastValue.length;i++) {
			lastValue[i] = 0d;
		}
	}
	
	/**
	 * Constructor
	 * Set limits on rate-of-change towards any target.
	 * Parameter to be limited must be a scalar.
	 * @param rateLimit
	 */
	public RateLimiterND(double rateLimit) {
		this(1,rateLimit);
	}
	
	/**
	 * Constructor
	 * Sets limits on the rate-of-change separately for zero and non-zero targets.
	 * A target of zero requires all inputs to be below the threshold. I.e. the
	 * rate limit is not applied individually for each element of the array.
	 * @param N Length of the array to be processed
	 * @param rateLimit
	 * @param stoppingRateLimit
	 */
	public RateLimiterND(int N, double rateLimit, double stoppingRateLimit) {
		this(N, rateLimit);
		maxChange2Zero = stoppingRateLimit;
	}
	
	/**
	 * Constructor
	 * Sets limits on the rate-of-change separately for zero and non-zero targets.
	 * Parameter to be limited must be a scalar.
	 * @param rateLimit
	 */
	public RateLimiterND(double rateLimit, double stoppingRateLimit) {
		this(1,rateLimit,stoppingRateLimit);
	}
	
	/**
	 * Limit the rate of change of the input parameter
	 * @param in
	 * @return in_rateLimitedc
	 */
	public double[] limitRate(double... in) {
		double[] out = new double[lastValue.length];
		double lim;
		//Are all inputs below the threshold?
		boolean isZero = true;
		for (int i = 0;i < lastValue.length;i++) {
			//If any inputs are above the threshold, updated flag and quit
			if (Math.abs(in[i]) > zeroThreshold) {
				isZero = false;
				break;
			}
		}
		if (isZero) {
			//Decay to zero
			lim = maxChange2Zero;
		} else {
			//Limit rate towards target
			lim = maxChange;
		}
		
		for (int i=0;i<lastValue.length;i++) {
			out[i] = Discontinuities.limit(in[i],lastValue[i]-lim,lastValue[i]+lim);
		}
		lastValue = out;
		return out;
	}
	
	/**
	 * Flush all previous values with zero
	 *
	 */
	public void flush() {
		//Preallocate with zeros
		for (int i=0; i < lastValue.length;i++) {
			lastValue[i] = 0d;
		}
	}
	
	/**
	 * Update the previous state values for the next iteration
	 * @param in
	 */
	public void setLastValue(double... in) {
		lastValue = in;
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
	 * TODO: what do I do if rateLimit =0? Disable? It will freeze the commands
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
	 * TODO: what do I do if rateLimit =0? Disable? It will freeze the commands
	 */
	public void setRateLimit(double rateLimit) {
		if (rateLimit < 0) {
			rateLimit = -rateLimit;
		}
		maxChange = rateLimit;
	}
		
}