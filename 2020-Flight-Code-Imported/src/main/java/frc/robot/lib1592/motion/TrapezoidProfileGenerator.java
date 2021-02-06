/*
 * Adapted from Team254 https://github.com/Team254/FRC-2017-Public/tree/master/src/com/team254/lib/util/motion
 */
package frc.robot.lib1592.motion;

import frc.robot.lib1592.motion.MotionProfileGoal.CompletionBehavior;

/**
 * A MotionProfileGenerator generates minimum-time MotionProfiles to travel from a given TrajectoryPoint to a given
 * MotionProfileGoal while obeying a set of MotionProfileConstraints.
 */
public class TrapezoidProfileGenerator {
    // Static class.
    private TrapezoidProfileGenerator() {}

    protected static MotionProfile generateFlippedProfile(double max_vel, double max_acc,
            MotionProfileGoal goal_state, TrajectoryPoint prev_state) {
        MotionProfile profile = generateProfile(max_vel, max_acc, goal_state.flipped(), prev_state.flipped());
        for (MotionSegment s : profile.segments()) {
            s.setStart(s.start().flipped());
            s.setEnd(s.end().flipped());
        }
        return profile;
    }

    /**
     * Generate a motion profile.
     * 
     * @param max_vel		Maximum velocity to accel to
     * @param max_acc		Acceleration when changing velocity
     * @param goal_state	Targeted pos, vel, and completion behavior
     * @param prev_state	The initial state to use.
     * @return A motion profile from prev_state to goal_state that satisfies constraints.
     */
    public synchronized static MotionProfile generateProfile(double max_vel, double max_acc, 
            MotionProfileGoal goal_state, TrajectoryPoint prev_state) {
        double delta_pos = goal_state.pos() - prev_state.pos();
        if (delta_pos < 0.0 || (delta_pos == 0.0 && prev_state.vel() < 0.0)) {
            // For simplicity, we always assume the goal requires positive movement. If negative, we flip to solve, then
            // flip the solution.
            return generateFlippedProfile(max_vel, max_acc, goal_state, prev_state);
        }
        // Invariant from this point on: delta_pos >= 0.0
        // Clamp the start state to be valid.
        TrajectoryPoint start_state = new TrajectoryPoint(prev_state.t(), prev_state.pos(),
        		prev_state.vel(), //Math.signum(prev_state.vel()) * Math.min(Math.abs(prev_state.vel()), max_vel),
                Math.signum(prev_state.acc()) * Math.min(Math.abs(prev_state.acc()), max_acc));
        MotionProfile profile = new MotionProfile();
        profile.reset(start_state);
        // If our velocity is headed away from the goal, the first thing we need to do is to stop.
        if (start_state.vel() < 0.0 && delta_pos > 0.0) {
            final double stopping_time = Math.abs(start_state.vel() / max_acc);
            profile.appendControl(max_acc, stopping_time);
            start_state = profile.endState();
            delta_pos = goal_state.pos() - start_state.pos();
        }
        // Invariant from this point on: start_state.vel() >= 0.0
        final double min_abs_vel_at_goal_sqr = start_state.vel2() - 2.0 * max_acc * delta_pos;
        final double min_abs_vel_at_goal = Math.sqrt(Math.abs(min_abs_vel_at_goal_sqr));
        final double max_abs_vel_at_goal = Math.sqrt(start_state.vel2() + 2.0 * max_acc * delta_pos);
        double goal_vel = goal_state.max_abs_vel();
        if (min_abs_vel_at_goal_sqr > 0.0
                && min_abs_vel_at_goal > (goal_state.max_abs_vel() + goal_state.vel_tolerance())) {
            // Overshoot is unavoidable with the current constraints. Look at completion_behavior to see what we should
            // do.
            if (goal_state.completion_behavior() == CompletionBehavior.ADJUST_VELOCITY) {
                // Adjust the goal velocity.
                goal_vel = min_abs_vel_at_goal;
            } else if (goal_state.completion_behavior() == CompletionBehavior.ADJUST_ACCEL) {
                if (Math.abs(delta_pos) < goal_state.pos_tolerance()) {
                    // Special case: We are at the goal but moving too fast. This requires 'infinite' acceleration,
                    // which will result in NaNs below, so we can return the profile immediately.
                    profile.appendSegment(new MotionSegment(
                            new TrajectoryPoint(profile.endTime(), profile.endPos(), profile.endState().vel(),
                                    Double.NEGATIVE_INFINITY),
                            new TrajectoryPoint(profile.endTime(), profile.endPos(), goal_vel, Double.NEGATIVE_INFINITY)));
                    profile.consolidate();
                    return profile;
                }
                // Adjust the max acceleration.
                max_acc = Math.abs(goal_vel * goal_vel - start_state.vel2()) / (2.0 * delta_pos);
            } else {
                // We are going to overshoot the goal, so the first thing we need to do is come to a stop.
                final double stopping_time = Math.abs(start_state.vel() / max_acc);
                profile.appendControl(-max_acc, stopping_time);
                // Now we need to travel backwards, so generate a flipped profile.
                profile.appendProfile(generateFlippedProfile(max_vel, max_acc, goal_state, profile.endState()));
                profile.consolidate();
                return profile;
            }
        } else {
        	//Limit goal velocity to max if we're not in an overshoot situation
        	goal_vel = Math.min(goal_vel, max_abs_vel_at_goal);
        }
        // Invariant from this point forward: We can achieve goal_vel at goal_state.pos exactly using no more than +/-
        // max_acc.
        
        //If we are going faster than vmax, decel to vmax
        if (start_state.vel() > Math.max(max_vel,goal_vel) + MotionUtil.EPSILON) {
        	final double accel_time = (start_state.vel() - Math.max(max_vel,goal_vel)) / max_acc;
            profile.appendControl(-max_acc, accel_time);
            start_state = profile.endState();
        }

        // Invariant from this point forward: We are moving less than max velocity
        
        // What is the maximum velocity we can reach (vPeak)? This is the intersection of two curves: one accelerating
        // towards the goal from profile.finalState(), the other coming from the goal at max vel (in reverse). If vPeak
        // is greater than max_vel, we will clamp and cruise.
        // Solve the following three equations to find vPeak (by substitution):
        // vPeak^2 = Vstart^2 + 2*a*d_accel
        // Vgoal^2 = vPeak^2 - 2*a*d_decel
        // delta_pos = d_accel + d_decel
        final double v_peak = Math.min(max_vel,Math.sqrt((start_state.vel2() + goal_vel * goal_vel) / 2.0 + delta_pos * max_acc));

        // Accelerate to v_peak
        if (v_peak > start_state.vel()) {
            final double accel_time = (v_peak - start_state.vel()) / max_acc;
            profile.appendControl(max_acc, accel_time);
            start_state = profile.endState();
        }
        // Figure out how much distance will be covered during deceleration.
        final double distance_decel = Math.max(0.0,
                (start_state.vel2() - goal_vel * goal_vel) / (2.0 * max_acc));
        final double distance_cruise = Math.max(0.0, goal_state.pos() - start_state.pos() - distance_decel);
        // Cruise at constant velocity.
        if (distance_cruise > 0.0 + MotionUtil.EPSILON) {
            final double cruise_time = distance_cruise / start_state.vel();
            profile.appendControl(0.0, cruise_time);
            start_state = profile.endState();
        }
        // Decelerate to goal velocity.
        if (distance_decel > 0.0 + MotionUtil.EPSILON) {
            final double decel_time = (start_state.vel() - goal_vel) / max_acc;
            profile.appendControl(-max_acc, decel_time);
        }

        profile.finalize();
        return profile;
    }
}
