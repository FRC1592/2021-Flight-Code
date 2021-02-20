package frc.robot.lib1592.control;

import static java.util.Objects.requireNonNull;

import java.util.TimerTask;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;

/**
 * PID Interface Container Class
 * <p>
 * This class itself serves no purpose other than for the packaging of
 * the various interface groups
 */
public class InterfacePID {
	
	
	
	//==========================//
	//      PID Interface       //
	//==========================//
	
	/**
	 * Complete PID Interface
	 * <p>
	 * This interface provides complete access to a PID with full control
	 * over the object and its behavior.
	 */
	public static interface PID extends ChildPID, AbstractPID {}

	
	
	//==================================//
	//      AbstractPID Interface       //
	//==================================//
	
	/**
	 * Abstract PID Interface
	 * <p>
	 * This interface provides the methods necessary to target, control, and run a PID. 
	 * This interface is a subset of PID which does not require the class to be an 
	 * actual PID implementation itself and is usually used for classes which may 
	 * encapsulate one or more full PID objects.
	 */
	public static interface AbstractPID extends Targetable, Actionable {}

	
	
	//===============================//
	//      ChildPID Interface       //
	//===============================//
	
	/**
	 * Child PID Interface
	 * <p>
	 * This interface provides the methods necessary to define and target a PID. 
	 * This interface is a subset of PID which does not provide the methods to
	 * actually control the PID calculations, its enabled state, or its loop
	 * timing and is usually used for objects stored inside an AbstractPID and
	 * exposed externally. 
	 */
	public static interface ChildPID extends ParametricPID, ResultPID, Targetable {}
	
	
	
	//=============================//
	//      ND Source/Output       //
	//=============================//
	
	/**
	 * N-D version of PIDOutput
	 */
	public static interface PIDOutputND {

	  /**
	   * Set the output to the value(s) calculated by PID.
	   *
	   * @param output the value(s) calculated by PID
	   */
	  void pidWrite(double... output);
	}
	
	
	
	//================================//
	//      Tolerance Interface       //
	//================================//

	/**
	 * Tolerance is the type of tolerance used to specify if an object is on target. This is a 
	 * functional interface for simplified construction, if necessary. It is highly recommended,
	 * though not required, that concrete implementations of this class be immutable upon
	 * construction.
	 */
	public static interface Tolerance {
		
		/**
		 * Returns a value specifying whether the object is on target based
		 * on the definition of the Tolerance object.
		 *
		 * @return {@code true} if the object is considered by this object to be on target, {@code false} otherwise
		 */
		boolean onTarget();
	}
	
	
	
	//=====================================//
	//      Tolerance Implementations      //
	//=====================================//

	/** Percentage Error Based Implementation */
	public static class PercentageTolerance implements Tolerance {
		private final ChildPID pid;
		private final double m_percentage;

		/**
		 * Standard Constructor
		 * <p>
		 * It is important to recognize that this tolerance specification is unlikely to work successfully when either
		 * endpoint of the {@link PIDBase#getInputRange()} is infinite, which is the default.
		 *
		 * @param value  the percentage of the input range that specified the bound below which the absolute value of the error will be on target.
		 * 				 The value is expected to be in the range of [0, 100] and will be clamped to that if out of bounds. A zero value, either
		 * 				 input or clamped to, will result in the on target being true only when the error is exactly zero.
		 */
		public PercentageTolerance(ChildPID pid, double value) {
			this.pid = pid;
			value = clamp(value, 0, 100);
			m_percentage = value;
		}

		@Override
		public boolean onTarget() {
			double[] irange = pid.getInputRange();
			return Math.abs(pid.getError()) <= m_percentage / 100 * (irange[1]-irange[0]);
		}
	}

	/** Absolute Error Based Implementation */
	public static class AbsoluteTolerance implements Tolerance {
		private final ResultPID pid;
		private final double m_value;

		/**
		 * Standard Constructor
		 *
		 * @param value  the bound below which the absolute value of the error will be on target. The value is expected in the range
		 * 				 of [0, inf) and will be clamped to that if out of bounds. A zero value, either input or clamped to, will 
		 * 				 result in the on target being true only when the error is exactly zero.
		 */
		public AbsoluteTolerance(ResultPID pid, double value) {
			this.pid = pid;
			value = clamp(value, 0, Double.POSITIVE_INFINITY);
			m_value = value;
		}

		@Override
		public boolean onTarget() {
			return Math.abs(pid.getError()) <= m_value;
		}
	}
	
	
	
	//=================================//
	//      Targetable Interface       //
	//=================================//

	/**
	 * Targetable Interface
	 * <p>
	 * This interface simply provides the interface for using a Tolerance with an
	 * object and a method for determining if the object is on target.
	 */
	public static interface Targetable {
		
		/**
		 * Returns the Tolerance object used to determine whether this object is on target.
		 * <p>
		 * This value can be null, in which case this object should be assumed to never be
		 * on target.
		 */
		public Tolerance getTolerance();
		
		/**
		 * Returns whether this object is considered to be on target.
		 * <p>
		 * The default implementation returns false if {@link #getTolerance()} returns null.
		 * If an actual Tolerance object is returned, then this method returns the value
		 * of {@link Tolerance#onTarget()}.
		 */
		public default boolean onTarget() {
			return getTolerance() == null ? false : getTolerance().onTarget();
		}
		
		/**
		 * Sets the Tolerance object used to determine whether this object is on target.
		 * <p>
		 * This value can be set to null which implies that {@link #onTarget()} will
		 * always return false.
		 *
		 * @param tolerance  the Tolerance object or null
		 */
		public void setTolerance(Tolerance tolerance);
		
	}
	
	
	
	//=================================//
	//      Outputable Interface       //
	//=================================//
	
	private static interface Outputable {
		
		/** Returns the sink for the PID output */
		public PIDOutput getOutput();
		
		/**
		 * Sets the sink for the PID output
		 *
		 * @param output  the PID output sink
		 * @throws NullPointerException if {@code sink} is null
		 */
		public void setOutput(PIDOutput output);
		
	}
	
	
	
	//====================================//
	//      ParametricPID Interface       //
	//====================================//
	
	/**
	 * PID Parameteric Interface
	 * <p>
	 * This interface allows for control of all of the PID parameters defining the behavior
	 * of the PID and its performance.
	 */
	private static interface ParametricPID extends Outputable {

		/** Returns the finite proportional coefficient of the PID */
		public double getP();

		/** Returns the finite integral coefficient of the PID */
		public double getI();

		/** Returns the finite differential coefficient of the PID */
		public double getD();

		/** Returns the finite feed-forward coefficient of the PID */
		public double getF();

		/**
		 * Returns the finite integral zone (maximum integral term) of the PID
		 * <p>
		 * The maximum allowable integral term implies the maximum value of the integral
		 * coefficient multiplied by the total accumulated error. If this maximum value
		 * is reached, the integral portion of the PID will stop accumulating additional 
		 * error until the integral term is reduced below the integral zone value. In
		 * addition to this value being finite, it is also guaranteed to be greater than
		 * zero.
		 */
		public double getIZone();

		/** 
		 * Returns the setpoint of the PID
		 * <p>
		 * This value is guaranteed to be finite and to lie inside of the range
		 * specified by {@link #getInputRange()}.
		 */
		public double getSetpoint();

		/** Returns the source of the PID input */
		public PIDSource getSource();

		/**
		 * Returns the expected input range of the PIDSource as a two element array
		 * (memory independent of the underlying object) with the first element being
		 * the minimum value and the second element the maximum value. 
		 * <p>
		 * The minimum value will always be less than the maximum value and neither 
		 * value will be NaN, though they may be infinite. The set values of the input 
		 * range do not actually impose any changes to the received values from the 
		 * PIDSource (i.e. if they are out of range, they are used as is). However, 
		 * this range is used to control the allowable values of the setpoint and the 
		 * setpoint value is guaranteed to lie within this range.
		 */
		public double[] getInputRange();

		/**
		 * Returns the output range of the PID (to {@link #get()} and the PIDOutput)
		 * as a two element array (memory independent of the underlying object) with 
		 * the first element being the minimum value and the second element the maximum 
		 * value. 
		 * <p>
		 * The minimum value will always be less than the maximum value and neither 
		 * value will be NaN, though they may be infinite. This range is used to control 
		 * the allowable values of the result and the result value is guaranteed to lie 
		 * within this range. The result value is published to the PIDOutput and can be
		 * retrieved manually using {@link #get()}.
		 */
		public double[] getOutputRange();

		/**
		 * Returns whether to treat the PID as continuous (i.e circular).
		 * <p>
		 * A continuous PID treats the input range (i.e. {@link #getInputRange()}) as
		 * circular such that the minimum and maximum values are the same point. A
		 * continuous PID allows for the error to be the minimum distance from the
		 * setpoint to be calculated in either direction around the circle instead of
		 * only in a strictly linear manner.
		 * <p>
		 * If either value of input range is infinite, the PID value will not be
		 * considered continuous (and this method will return false) even if set
		 * to true using the {@link #setContinuous(boolean)}.
		 */
		public boolean isContinuous();
		
		/**
		 * Sets the finite proportional coefficient of the PID
		 *
		 * @param kP  the proportional coefficient
		 * @throws IllegalArgumentException if {@code kP} is not finite
		 */
		public void setP(double kP);
		
		/**
		 * Sets the finite integral coefficient of the PID
		 *
		 * @param kI  the integral coefficient
		 * @throws IllegalArgumentException if {@code kI} is not finite
		 */
		public void setI(double kI);
		
		/**
		 * Sets the finite differential coefficient of the PID
		 *
		 * @param kD  the differential coefficient
		 * @throws IllegalArgumentException if {@code kD} is not finite
		 */
		public void setD(double kD);
		
		/**
		 * Sets the finite feed-forward coefficient of the PID
		 *
		 * @param kF  the feed-forward coefficient
		 * @throws IllegalArgumentException if {@code kF} is not finite
		 */
		public void setF(double kF);
		
		/**
		 * Sets the finite integral zone (maximum integral term) of the PID
		 * <p>
		 * The maximum allowable integral term implies the maximum value of the integral
		 * coefficient multiplied by the total accumulated error. If this maximum value
		 * is reached, the integral portion of the PID will stop accumulating additional 
		 * error until the integral term is reduced below the integral zone value.
		 *
		 * @param iZone  the finite integral zone
		 * @throws IllegalArgumentException if {@code iZone} is not finite or less than or equal to zero
		 */
		public void setIZone(double iZone);
		
		/**
		 * Sets the setpoint of the PID
		 * <p>
		 * If the input setpoint value lies outside the range given by {@link #getInputRange()},
		 * the value is clamped to the range prior to being set as the actual setpoint. The
		 * clamping methodology simply forces all input values less than the minimum of the
		 * range to the minimum value and all values greater than the maximum of the range
		 * to the maximum value.
		 *
		 * @param setpoint  the setpoint value
		 * @return the actual setpoint value set by this method after clamping (if necessary)
		 * @throws IllegalArgumentException if {@code setpoint} is not finite
		 */
		public double setSetpoint(double setpoint);
		
		/**
		 * Sets the source of the PID input
		 *
		 * @param source  the PID input source
		 * @throws NullPointerException if {@code source} is null
		 */
		public void setSource(PIDSource source);
		
		/**
		 * Sets the expected input range of the PIDSource.
		 * <p>
		 * The set values of the input range do not actually impose any changes to the 
		 * received values from the PIDSource (i.e. if they are out of range, they are used 
		 * as is). However, this range is used to control the allowable values of the setpoint 
		 * and the setpoint value is guaranteed to lie within this range.
		 * 
		 * @param min  the minimum value of the expected input range
		 * @param max  the maximum value of the expected input range
		 * @throws IllegalArgumentException if {@code min} or {@code max} is NaN or {@code min >= max}
		 */
		public void setInputRange(double min, double max);
		
		/**
		 * Set the output range of the PID (to {@link #get()} and the PIDOutput).
		 * <p>
		 * This range is used to control the allowable values of the result and the result 
		 * value is guaranteed to lie within this range. The result value is published to 
		 * the PIDOutput and can be retrieved manually using {@link #get()}.
		 * 
		 * @param min  the minimum value of the output
		 * @param max  the maximum value of the output
		 * @throws IllegalArgumentException if {@code min} or {@code max} is NaN or {@code min >= max}
		 */
		public void setOutputRange(double min, double max);

		/**
		 * Sets whether to treat the PID as continuous (i.e circular).
		 * <p>
		 * A continuous PID treats the input range (i.e. {@link #getInputRange()}) as
		 * circular such that the minimum and maximum values are the same point. A
		 * continuous PID allows for the error to be the minimum distance from the
		 * setpoint to be calculated in either direction around the circle instead of
		 * only in a strictly linear manner.
		 * <p>
		 * If either value of input range is infinite, the PID is considered not to
		 * be continuous and usage of the method is considered to be a no-op until
		 * such time as the input range is completely finite.
		 *
		 * @param continuous  whether to treat the PID as continuous (i.e circular).
		 */
		public void setContinuous(boolean continuous);
	
	}
	
	
	
	//================================//
	//      ResultPID Interface       //
	//================================//
	
	/**
	 * Result PID Interface
	 * <p>
	 * This interface provides direct access to the PID results of calculation
	 * and the current error of the object.
	 */
	private static interface ResultPID {
		
		/**
		 * Returns the current result value of the PID.
		 * <p>
		 * This value will always lie within the range returned by {@link #getOutputRange()}
		 * and is guaranteed to be finite.
		 */
		public double get();
		
		/**
		 * Returns the current difference of the input value from the setpoint while
		 * properly handling if the PID is continuous or not.
		 */
		public double getError();
		
	}
	
	
	
	//=================================//
	//      Actionable Interface       //
	//=================================//

	/**
	 * Actionable Interface
	 * <p>
	 * This interface allows for PID control using a calculate method as well as access to the enable 
	 * and disable control and the loop timing as well as including the result access.
	 */
	private static interface Actionable extends ResultPID, Outputable {
		
		/**
		 * Returns the period (in milliseconds) between subsequent calls to {@link #calculate()}
		 * when this PID is enabled (i.e {@link #isEnabled()} is true). Those calls usually
		 * occur on a background thread. This value is gauranteed to be greater than zero.
		 */
		public long getPeriod();
		
		/** Returns whether the PID is enabled */
		public boolean isEnabled();
		
		/**
		 * Sets the period (in milliseconds) between subsequent calls to {@link #calculate()}
		 * when this PID is enabled (i.e {@link #isEnabled()} is true). Those calls usually
		 * occur on a background thread.
		 *
		 * @param period  the period (in milliseconds) between subsequent calls to {@link #calculate()}
		 * @throws IllegalArgumentException if {@code period <= 0}
		 */
		public void setPeriod(long period);
		
		/**
		 * Sets whether the PID is enabled
		 * 
		 * @param enabled  whether the PID is enabled
		 */
		public void setEnabled(boolean enabled);
		
		/**
		 * Calculation method
		 * <p>
		 * This method performs the main calculation of the PID by acquiring an input from the PIDSource,
		 * performing all the necessary calculations using the PID settings, and updating the result
		 * value by both publishing to the PIDOutput and storing locally for access with {@link #get()}.
		 * <p>
		 * This method, while available in the interface for subclasses, is generally not recommended
		 * to be called externally as changing the timing frequency of the calculate calls will alter the
		 * performance of both the integral and differential terms.
		 */
		public void calculate();
		
	}
	
	
	//====================//
	//      PID Task      //
	//====================//
	
	/**
	 * Actionable Wrapper As a Timer Task
	 */
	public static class ActionableTask extends TimerTask {
		private Actionable action;

		ActionableTask(Actionable action) {
			requireNonNull(action, "Given PID was null");
			this.action = action;
		}

		@Override public void run() {
			if (action.isEnabled()) {
				action.calculate();
			}
		}
	}



	//=================================================//
	//      Package Private Static Helper Methods      //
	//=================================================//

	/**
	 * Clamp Method
	 * <p>
	 * This method uses {@link Math#max(double, double)} and {@link Math#min(double, double)} to
	 * return a value within the specified low to high range. The value is the original value if
	 * the 
	 *
	 * @param value
	 * @param low
	 * @param high
	 * @return
	 */
	static double clamp(double value, double low, double high) {
		return Math.max(low, Math.min(value, high));
	}

}
