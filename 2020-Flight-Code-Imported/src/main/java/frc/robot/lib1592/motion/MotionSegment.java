/*
 * Adapted from Team254 https://github.com/Team254/FRC-2017-Public/tree/master/src/com/team254/lib/util/motion
 */
package frc.robot.lib1592.motion;

/**
 * A MotionSegment is a movement from a start TrajectoryPoint to an end TrajectoryPoint with a constant acceleration.
 */
public class MotionSegment {
    protected TrajectoryPoint mStart;
    protected TrajectoryPoint mEnd;

    public MotionSegment(TrajectoryPoint start, TrajectoryPoint end) {
        mStart = start;
        mEnd = end;
    }

    /**
     * Verifies that:
     * 
     * 1. All segments have a constant acceleration.
     * 
     * 2. All segments have monotonic position (sign of velocity doesn't change).
     * 
     * 3. The time, position, velocity, and acceleration of the profile are consistent.
     */
    public boolean isValid() {
        if (!MotionUtil.epsEquals(start().acc(), end().acc(), MotionUtil.EPSILON)) {
            // Acceleration is not constant within the segment.
            System.err.println(
                    "Segment acceleration not constant! Start acc: " + start().acc() + ", End acc: " + end().acc());
            return false;
        }
        if (Math.signum(start().vel()) * Math.signum(end().vel()) < 0.0 && !MotionUtil.epsEquals(start().vel(), 0.0, MotionUtil.EPSILON)
                && !MotionUtil.epsEquals(end().vel(), 0.0, MotionUtil.EPSILON)) {
            // Velocity direction reverses within the segment.
            System.err.println("Segment velocity reverses! Start vel: " + start().vel() + ", End vel: " + end().vel());
            return false;
        }
        if (!start().extrapolate(end().t()).equals(end())) {
            // A single segment is not consistent.
            if (start().t() == end().t() && Double.isInfinite(start().acc())) {
                // One allowed exception: If acc is infinite and dt is zero.
                return true;
            }
            System.err.println("Segment not consistent! Start: " + start() + ", End: " + end());
            return false;
        }
        return true;
    }

    public boolean containsTime(double t) {
        return t >= start().t() && t <= end().t();
    }

    public boolean containsPos(double pos) {
        return pos >= start().pos() && pos <= end().pos() || pos <= start().pos() && pos >= end().pos();
    }

    public TrajectoryPoint start() {
        return mStart;
    }

    public void setStart(TrajectoryPoint start) {
        mStart = start;
    }

    public TrajectoryPoint end() {
        return mEnd;
    }

    public void setEnd(TrajectoryPoint end) {
        mEnd = end;
    }
    
//    public void setReferenceTime(double refTime) {
//    	shiftTime(refTime - mStart.t);
//    }
    
//    public void shiftTime(double deltaTime) {
//    	mStart = mStart.shiftTime(deltaTime);
//    	mEnd = mEnd.shiftTime(deltaTime);
//    }

    @Override
    public String toString() {
        return "Start: " + start() + ", End: " + end();
    }
}
