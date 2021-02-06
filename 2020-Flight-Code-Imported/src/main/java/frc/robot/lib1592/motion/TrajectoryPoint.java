package frc.robot.lib1592.motion;

import java.text.DecimalFormat;
import java.text.NumberFormat;

/**
 * A TrajectoryPoint is a completely specified state of 1D motion through time.
 */
public class TrajectoryPoint {
    protected final double t;
    protected final double pos;
    protected final double vel;
    protected final double acc;

    public static TrajectoryPoint kInvalidState = new TrajectoryPoint(Double.NaN, Double.NaN, Double.NaN, Double.NaN);

    /**
     * Construct a trajectory point with all states
     * 
     * @param t		time
     * @param pos	position
     * @param vel	velocity
     * @param acc	acceleration
     */
    public TrajectoryPoint(double t, double pos, double vel, double acc) {
        this.t = t;
        this.pos = pos;
        this.vel = vel;
        this.acc = acc;
    }
    
    /**
     * Construct a TrajectoryPoint at the origin
     */
    public TrajectoryPoint() {
    	this(0d,0d,0d,0d);
    }

    /**
     * Copy Constructor
     * 
     * @param state
     */
    public TrajectoryPoint(TrajectoryPoint state) {
        this(state.t, state.pos, state.vel, state.acc);
    }

    public double t() {
        return t;
    }

    public double pos() {
        return pos;
    }

    public double vel() {
        return vel;
    }

    public double vel2() {
        return vel * vel;
    }

    public double acc() {
        return acc;
    }

    /**
     * Extrapolates this TrajectoryPoint to the specified time by applying this TrajectoryPoint's acceleration.
     * 
     * @param t
     *            The time of the new TrajectoryPoint.
     * @return A TrajectoryPoint that is a valid predecessor (if t<=0) or successor (if t>=0) of this state.
     */
    public TrajectoryPoint extrapolate(double t) {
        return extrapolate(t, acc);
    }
    
//    /**
//     * Shift the time of a Trajectory Point.  This method returns a new point
//     * since the fields are final
//     * 
//     * @param dt  time to shift the point
//     * @return  new time shifted Trajectory point
//     */
//    public TrajectoryPoint shiftTime(double dt) {
//    	return new TrajectoryPoint(this.t+dt,this.pos,this.vel,this.acc);
//    }

    /**
     * 
     * Extrapolates this TrajectoryPoint to the specified time by applying a given acceleration to the (t, pos, vel) portion
     * of this TrajectoryPoint.
     * 
     * @param t
     *            The time of the new TrajectoryPoint.
     * @param acc
     *            The acceleration to apply.
     * @return A TrajectoryPoint that is a valid predecessor (if t<=0) or successor (if t>=0) of this state (with the
     *         specified accel).
     */
    public TrajectoryPoint extrapolate(double t, double acc) {
        final double dt = t - this.t;
        return new TrajectoryPoint(t, pos + vel * dt + .5 * acc * dt * dt, vel + acc * dt, acc);
    }

    /**
     * Find the next time (first time > TrajectoryPoint.t()) that this TrajectoryPoint will be at pos. This is an inverse of the
     * extrapolate() method.
     * 
     * @param pos
     *            The position to query.
     * @return The time when we are next at pos() if we are extrapolating with a positive dt. NaN if we never reach pos.
     */
    public double nextTimeAtPos(double pos) {
        if (MotionUtil.epsEquals(pos, this.pos, MotionUtil.EPSILON)) {
            // Already at pos.
            return t;
        }
        if (MotionUtil.epsEquals(acc, 0.0, MotionUtil.EPSILON)) {
            // Zero acceleration case.
            final double delta_pos = pos - this.pos;
            if (!MotionUtil.epsEquals(vel, 0.0, MotionUtil.EPSILON) && Math.signum(delta_pos) == Math.signum(vel)) {
                // Constant velocity heading towards pos.
                return delta_pos / vel + t;
            }
            return Double.NaN;
        }

        // Solve the quadratic formula.
        // ax^2 + bx + c == 0
        // x = dt
        // a = .5 * acc
        // b = vel
        // c = this.pos - pos
        final double disc = vel * vel - 2.0 * acc * (this.pos - pos);
        if (disc < 0.0) {
            // Extrapolating this TrajectoryPoint never reaches the desired pos.
            return Double.NaN;
        }
        final double sqrt_disc = Math.sqrt(disc);
        final double max_dt = (-vel + sqrt_disc) / acc;
        final double min_dt = (-vel - sqrt_disc) / acc;
        if (min_dt >= 0.0 && (max_dt < 0.0 || min_dt < max_dt)) {
            return t + min_dt;
        }
        if (max_dt >= 0.0) {
            return t + max_dt;
        }
        // We only reach the desired pos in the past.
        return Double.NaN;
    }

    /**
     * Checks if two MotionStates are epsilon-equals (all fields are equal within a nominal tolerance).
     */
    @Override
    public boolean equals(Object other) {
        return (other instanceof TrajectoryPoint) && equals((TrajectoryPoint) other, MotionUtil.EPSILON);
    }

    /**
     * Checks if two MotionStates are epsilon-equals (all fields are equal within a specified tolerance).
     */
    public boolean equals(TrajectoryPoint other, double epsilon) {
        return coincident(other, epsilon) && MotionUtil.epsEquals(acc, other.acc, epsilon);
    }

    /**
     * Checks if two MotionStates are coincident (t, pos, and vel are equal within a nominal tolerance, but acceleration
     * may be different).
     */
    public boolean coincident(TrajectoryPoint other) {
        return coincident(other, MotionUtil.EPSILON);
    }

    /**
     * Checks if two MotionStates are coincident (t, pos, and vel are equal within a specified tolerance, but
     * acceleration may be different).
     */
    public boolean coincident(TrajectoryPoint other, double epsilon) {
        return MotionUtil.epsEquals(t, other.t, epsilon) && MotionUtil.epsEquals(pos, other.pos, epsilon)
                && MotionUtil.epsEquals(vel, other.vel, epsilon);
    }

    /**
     * Returns a TrajectoryPoint that is the mirror image of this one. Pos, vel, and acc are all negated, but time is not.
     */
    public TrajectoryPoint flipped() {
        return new TrajectoryPoint(t, -pos, -vel, -acc);
	}
	
	@Override
	public String toString() {
		NumberFormat formatter = new DecimalFormat("#0.000");
		return  formatter.format(t) + ", " + formatter.format(pos) + ", " + formatter.format(vel) + ", " + formatter.format(acc);
	}
	
}
