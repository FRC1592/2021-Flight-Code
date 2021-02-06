package frc.robot.lib1592.control;

import frc.robot.lib1592.control.InterfacePID.Tolerance;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

/**
 * PID Builder
 * <p>
 * This class serves as a simple container for variables used to construct
 * one or more separate versions of the various PID interface types. This
 * variable encapsulation serves to both provide a more streamlined input
 * process as well as provide for input reusability. The usage of all of
 * the variables in the construction of a given type is defined by that
 * type including what values are acceptable for any variable, what values
 * cannot be null, and what values are ignored.
 */
public class BuilderPID {
	
	//=======================//
	//      Properties       //
	//=======================//
	
	public static final long kDefaultPeriod = 50;		// Default Loop Period (milliseconds)
	
	public double kP = 0;			// Proportional Coefficient
	public double kI = 0;			// Integral Coefficient
	public double kD = 0;			// Differential Coefficient
	public double kF = 0;			// Feed-forward Coefficient
	public double iZone = 1;		// Maximum Integral Term
	public double setpoint = 0;		// Setpoint Value
	
	public PIDSource source = null;		// Source
	public PIDOutput output = null;		// Sink
	
	public double minimumInput = Double.NEGATIVE_INFINITY;		// Input (Setpoint) Minimum
	public double maximumInput = Double.POSITIVE_INFINITY;		// Input (Setpoint) Maximum
	public double minimumOutput = -1;							// Output Minimum
	public double maximumOutput = 1;							// Output Maximum
	public boolean continuous = false;							// Continuous PID
	
	public Tolerance tolerance = null;		// Tolerance Target
	
	public long period = kDefaultPeriod;	// Loop Period (milliseconds)
	public boolean enabled = false;			// Enabled PID On Startup
	
	
	
	//=========================//
	//      Constructors       //
	//=========================//
	
	/**
	 * Default Constructor
	 */
	public BuilderPID() {}
	
	/**
	 * Source/Sink Constructor
	 *
	 * @param source  the pid source
	 * @param output  the pid output sink
	 */
	public BuilderPID(PIDSource source, PIDOutput output) {
		this(source, output, null);
	}
	
	/**
	 * Basic Constructor
	 *
	 * @param source  the pid source
	 * @param output  the pid output sink
	 * @param constants  the pid constants (P, I, D, F, iZone, and period), ignored if null
	 */
	public BuilderPID(PIDSource source, PIDOutput output, PIDConstants constants) {
		if (constants != null) {
			this.kP = constants.kP;
			this.kI = constants.kI;
			this.kD = constants.kD;
			this.kF = constants.kF;
			this.iZone = constants.iZone;
			this.period = Double.isFinite(constants.period) ? ((long) constants.period * 1000) : kDefaultPeriod;
			this.minimumOutput = constants.minOutput;
			this.maximumOutput = constants.maxOutput;
		}
		this.source = source;
		this.output = output;
	}
	
	
	
	//====================//
	//      Methods       //
	//====================//
	
	/**
	 * Returns a copy of the current builder.
	 * <p>
	 * The new builder object starts in the same state as the current value as all values
	 * are copied from the current builder to the new builder. The resulting builder's
	 * independence form the current builder is dependent on the immutability of the
	 * source, output, and tolerance objects as those items are simply assigned to the
	 * new builder without any advanced copying.
	 *
	 * @return a new builder that is the copy of the current builder
	 */
	public BuilderPID copy() {
		BuilderPID out = new BuilderPID();
		out.kP = kP;
		out.kI = kI;
		out.kD = kD;
		out.kF = kF;
		out.iZone = iZone;
		out.setpoint = setpoint;
		out.source = source;
		out.output = output;
		out.minimumInput = minimumInput;
		out.maximumInput = maximumInput;
		out.minimumOutput = minimumOutput;
		out.maximumOutput = maximumOutput;
		out.continuous = continuous;
		out.tolerance = tolerance;
		out.period = period;
		out.enabled = enabled;
		return out;
	}
	
	/**
	 * Returns whether the current builder is intrinsically valid.
	 * <p>
	 * To be valid, the following conditions must be met:
	 * <ul>
	 * <li>{@code kP}, {@code kI}, {@code kD}, {@code kF}, {@code iZone}, and {@code setpoint} are finite</li>
	 * <li>{@code iZone > 0}</li>
	 * <li>{@code source} and {@code output} are not null</li>
	 * <li>{@code minimumInput}, {@code maximumInput}, {@code minimumOutput}, and {@code maximumOutput} are not NaN</li>
	 * <li>{@code minimumInput < maximumInput}</li>
	 * <li>{@code minimumOutput < maximumOutput}</li>
	 * <li>{@code minimumInput <= setpoint <= maximumInput}</li>
	 * <li>{@code period > 0}</li>
	 * <li>if {@code continuous = true}, then {@code minimumInput} and {@code maximumInput} must be finite</li>
	 * </ul>
	 *
	 * @return whether the builder is currently valid based on the criteria
	 */
	public boolean isValid() {
		return Double.isFinite(kP) && Double.isFinite(kI) && Double.isFinite(kD) && Double.isFinite(kF) && Double.isFinite(iZone) && Double.isFinite(setpoint) &&
				iZone > 0 &&
				source != null && output != null &&
				!Double.isNaN(minimumInput) && !Double.isNaN(maximumInput) && !Double.isNaN(minimumOutput) && !Double.isNaN(maximumOutput) &&
				minimumInput < maximumInput &&
				minimumOutput < maximumOutput &&
				minimumInput <= setpoint && setpoint <= maximumInput &&
				period > 0 &&
				(continuous == false || (Double.isFinite(minimumInput) && Double.isFinite(maximumInput)));
	}
	
	/**
	 * Makes the builder valid by correcting any conditions of {@link #isValid()} that are not met.
	 * <p>
	 * The conditions are corrected in a manner as to reduce the number of changes and to reduce
	 * the distance between the logical state of the current builder and the logical state of the result.
	 * <p>
	 * The changes proceed in the following manner:
	 * <ul>
	 * <li>{@code kP}, {@code kI}, {@code kD}, and {@code kF} are converted to 0.0 if not finite</li>
	 * <li>{@code iZone} is converted to 1.0 if not finite or if {@code iZone <= 0}</li>
	 * <li>{@code source} is a assigned an initially kRate source that always returns zero if it is null</li>
	 * <li>{@code output} is a assigned a complete consumer (no action) if it is null</li>
	 * <li>{@code minimumInput} and {@code minimumOutput} are set to negative infinity if they are NaN</li>
	 * <li>{@code maximumInput} and {@code maximumOutput} are set to positive infinity if they are NaN</li>
	 * <li>{@code minimumInput} and {@code maximumInput} are swapped if {@code minimumInput > maximumInput}</li>
	 * <li>{@code minimumInput} is set to negative infinity if {@code minimumInput == maximumInput}</li>
	 * <li>{@code minimumOutput} and {@code maximumOutput} are swapped if {@code minimumOutput > maximumOutput}</li>
	 * <li>{@code minimumOutput} is set to negative infinity if {@code minimumOutput == maximumOutput}</li>
	 * <li>{@code period} is set to 50 if {@code period <= 0}</li>
	 * <li>{@code continuous} is set to false if {@code continuous} is true and {@code minimumInput} or {@code maximumInput} is not finite</li>
	 * </ul>
	 * <p>
	 * If the builder is currently valid (i.e {@link #isValid()} is true), no changes are made.
	 */
	public void makeValid() {
		if (isValid()) {return;}
		if (!Double.isFinite(kP)) { kP = 0; }
		if (!Double.isFinite(kI)) { kI = 0; }
		if (!Double.isFinite(kD)) { kD = 0; }
		if (!Double.isFinite(kF)) { kF = 0; }
		if (!Double.isFinite(iZone) || iZone <= 0) { iZone = 1; }
		if (source == null) { source = zeroSource; }
		if (output == null) { output = emptySink; }
		if (Double.isNaN(minimumInput)) { minimumInput = Double.NEGATIVE_INFINITY; }
		if (Double.isNaN(minimumOutput)) { minimumOutput = Double.NEGATIVE_INFINITY; }
		if (Double.isNaN(maximumInput)) { maximumInput = Double.POSITIVE_INFINITY; }
		if (Double.isNaN(maximumOutput)) { maximumOutput = Double.POSITIVE_INFINITY; }
		if (minimumInput > maximumInput) { 
			double temp = minimumInput;
			minimumInput = maximumInput;
			maximumInput = temp;
		} else if (minimumInput == maximumInput) {
			minimumInput = Double.NEGATIVE_INFINITY;
		}
		if (minimumOutput > maximumOutput) { 
			double temp = minimumOutput;
			minimumOutput = maximumOutput;
			maximumOutput = temp;
		} else if (minimumOutput == maximumOutput) {
			minimumOutput = Double.NEGATIVE_INFINITY;
		}
		if (period <= 0) { period = 50; }
		if (continuous == true && (!Double.isFinite(minimumInput) || !Double.isFinite(maximumInput))) { continuous = false; }
	}
	
	

	//=========================================//
	//      Internal Source/Sink Classes       //
	//=========================================//

	private static final PIDSource zeroSource = new ZeroSource();
	static final PIDOutput emptySink = (v) -> {};

	/** Source (kRate initially) Providing Zero Constantly */
	private static class ZeroSource implements PIDSource {
		private PIDSourceType type = PIDSourceType.kRate;

		@Override public void setPIDSourceType(PIDSourceType pidSource) {
			if (pidSource == null) {throw new NullPointerException();}
			type = pidSource;
		}

		@Override public PIDSourceType getPIDSourceType() { return type; }

		@Override public double pidGet() { return 0; }
		
	}
	
}
