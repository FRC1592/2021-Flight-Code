package frc.robot.lib1592.motion;

/**
 * Rotation behavior modes control the timing of when a turn happens in a PathSegment
 * if the turn takes less time than driving.
 * 
 * - Early: 	Start the turn as soon as you start moving on the path segment
 * 
 * - Late:		Wait as late as possible to start the turn while allowing enough time
 * 				to finish subject to the TrajConstraints
 * 
 * - MIN_VEL: 	Reduce the max velocity constraint such that the turn will take the full
 *  			duration of the path segment
 *  
 * - MIN_ACCEL:	Reduce the acceleration rate such that the turn will take the full duration
 *  			of the path segment.
 * 
 * @author dedyer1
 *
 */
public enum RotateBehavior {
	EARLY,
	LATE,
	MIN_VEL,
	MIN_ACCEL;
}
