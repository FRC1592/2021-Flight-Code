package frc.robot.lib1592.control;

import java.util.concurrent.atomic.AtomicInteger;

import frc.robot.lib1592.control.InterfacePID.PID;
import frc.robot.lib1592.control.InterfacePID.Tolerance;

// import edu.wpi.first.wpilibj.HLUsageReporting;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.SendableBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

/**
 * PID Base Class
 */
class PIDBase extends SendableBase implements PID {

	//======================//
	//      Properties      //
	//======================//
	
	// Static Instance Count
	private static AtomicInteger instances = new AtomicInteger(0);

	// Alterable Constant Values (No Defaults)
	private volatile double m_P; // factor for "proportional" control
	private volatile double m_I; // factor for "integral" control
	private volatile double m_D; // factor for "derivative" control
	private volatile double m_F; // factor for feedforward term
	private volatile double m_iZone; // maximum control output from integrator

	// Fixed Source and Sink
	protected volatile PIDSource m_pidInput;
	protected volatile PIDOutput m_pidOutput;

	// Alterable Constant Values (With Defaults)
	private volatile double m_maximumOutput = 1.0; // |maximum output|
	private volatile double m_minimumOutput = -1.0; // |minimum output|
	private volatile double m_maximumInput = Double.POSITIVE_INFINITY; // maximum input - limit setpoint to this
	private volatile double m_minimumInput = Double.NEGATIVE_INFINITY; // minimum input - limit setpoint to this
	private volatile double m_setpoint = 0.0;
	private volatile Tolerance m_tolerance = null; // the tolerance object used to check if on target
	private volatile long m_period = 50;

	// Boolean Settings
	private volatile boolean m_enabled = false;
	private volatile boolean m_continuous = false;

	// Internal Storage Values (Non-volatile because only updated in synchronization blocks)
	private double m_prevInput = 0.0;
	private double m_accumError = 0.0;
	private boolean m_isFirstPass = true;
	
	// Result Value
	private volatile double m_result = 0.0;



	//=======================//
	//      Constructor      //
	//=======================//
	
	/**
	 * Create a full PID base object using the supplied builder.
	 * <p>
	 * If the builder is not valid (i.e {@link BuilderPID#isValid()} returns false), the
	 * builder is copied using {@link BuilderPID#copy()} and then made valid using
	 * {@link BuilderPID#makeValid()}.
	 *
	 * @param builder  the object on which to base the created PID
	 * @throws NullPointerException if {@code builder} is null
	 */
	public PIDBase(BuilderPID builder) {
		super(false);
		if (builder == null) {throw new NullPointerException();}
		if (!builder.isValid()) {
			builder = builder.copy();
			builder.makeValid();
		}
		
		m_P = builder.kP;
		m_I = builder.kI;
		m_D = builder.kD;
		m_F = builder.kF;
		m_iZone = builder.iZone;
		
		m_pidInput = builder.source;
		m_pidOutput = builder.output;
		
		m_minimumInput = builder.minimumInput;
		m_maximumInput = builder.maximumInput;
		m_minimumOutput = builder.minimumOutput;
		m_maximumOutput = builder.maximumOutput;
		m_setpoint = builder.setpoint;
		m_tolerance = builder.tolerance;
		m_period = builder.period;
		
		m_enabled = builder.enabled;
		m_continuous = builder.continuous;
		
		synchronized (instances) {
			// HLUsageReporting.reportPIDController(instances.incrementAndGet()); //TODO: Update for 2019
			setName("PIDController", instances.get());
		}
	}



	//========================================//
	//      Public Final Calculate Method     //
	//========================================//

	@Override public final void calculate() {
		synchronized (this) {
			if (m_pidInput == null || m_pidOutput == null || !isEnabled()) {
				return;
			}
	
			// Access Members
			double input = m_pidInput.pidGet();
			double P = m_P;
			double I = m_I;
			double D = m_D;
			double F = calculateFeedForward();
			double minimumOutput = m_minimumOutput;
			double maximumOutput = m_maximumOutput;
			if (m_isFirstPass) {
				m_prevInput = input;
				m_isFirstPass = false;
			}
			double prevInput = m_prevInput;
			double error = getContinuousError(m_setpoint - input);
			//NOTE: 2DOF PID ignores the cmd when computing the derivative contribution to prevent
			//transients when the command steps
			double dError = getContinuousError(input - prevInput);
			double derivativeTerm = -D * dError;
			double accumError = accumulateError(m_accumError, error, derivativeTerm, F);
	
			// Calculate Result
			double result = P * error + I * accumError + derivativeTerm + F;
			//FIXME: TEMP DEBUG
//			if (result < minimumOutput || result > maximumOutput ) {
//				//TEMP: print debug results
//				System.out.println("Cmd " + m_setpoint);
//				System.out.println("Actual (Now, Prev, Delta) " + input + ", " + prevInput + ", " + dError);
//				System.out.println("Error (Now, Accum) " + error + ", " + accumError);
//				System.out.println("PIDF " + (P * error) + ", " + (I * accumError) + ", " + derivativeTerm + ", " + F);
//				System.out.println("Result " + result);
//			}
			result = InterfacePID.clamp(result, minimumOutput, maximumOutput);
	
			// Store Calculation Update
			m_pidOutput.pidWrite(result);
			m_prevInput = input;
			m_accumError = accumError;
			m_result = result;
		}
	}



	//=======================================================//
	//      Protected Internal Calculate Helper Methods      //
	//=======================================================//

	/**
	 * Calculates the updated total accumulated error based on the input values. Any other required values can be
	 * accessed safely using their getter methods as this method is called inside {@link #calculate()} while inside
	 * of a synchronized block.
	 *
	 * @param totalError  the total accumulated error prior to this method call
	 * @param error  the current error as calculated using the input value, the current set point, and whether the PID is continous
	 * @param derivativeTerm  derivative contribution to the controller output
	 * @param F  the calculated feed forward term to be used in the PIDBase {@link #calculate()} method
	 * @return the resulting accumulated total error term
	 */
	protected double accumulateError(double totalError, double error, double derivativeTerm, double F) {
		if(getI() != 0) {
			//Calculated the integral contribution after accumulating more error
			double potentialIOutput = (totalError + error) * getI();
			//If integral contribution will exceed iZone, do not accumulate more error
			if (getIZone() == 0 || Math.abs(potentialIOutput) < getIZone()) {
				double potentialOutput = potentialIOutput + getP() * error + derivativeTerm + F;
				double[] rangeOutput = getOutputRange();
				//Clamp the integrator if the output is saturated.  Otherwise, allow the integrated error to decay back to zero
				boolean outputNotSaturated = potentialOutput < rangeOutput[1] && potentialOutput > rangeOutput[0]; 
				if (outputNotSaturated) {
					totalError += error;
				} else if (error > 0 && totalError < 0){
					totalError = Math.min(totalError + error, 0);
				} else if (error < 0 && totalError > 0) {
					totalError = Math.max(totalError + error, 0);
				}
			} 
		}
		return totalError;
	}

	/**
	 * Calculate the feed forward term for use in {@link #calculate()}.
	 */
	protected double calculateFeedForward() {
		return getF() * getSetpoint();
	}
	
	
	
	//==========================//
	//      Enable/Disable      //
	//==========================//
	
	@Override public void setEnabled(boolean enable) {
		if (enable) {
			synchronized (this) {
				m_enabled = true;
			}
		} else {
			synchronized (this) {
				m_enabled = false;
				m_pidOutput.pidWrite(0);
				//Reset flag so next time through will be a first pass
				m_isFirstPass = true;
			}
		}
	}

	@Override public boolean isEnabled() {
		return m_enabled;
	}



	//===================//
	//      Getters      //
	//===================//
	
	@Override public long getPeriod() {
		return m_period;
	}

	@Override public double getP() {
		return m_P;
	}

	@Override public double getI() {
		return m_I;
	}

	@Override public double getD() {
		return m_D;
	}

	@Override public double getF() {
		return m_F;
	}

	@Override public double getIZone() {
		return m_iZone;
	}
	
	@Override public PIDSource getSource() {
		return m_pidInput;
	}

	@Override public PIDOutput getOutput() {
		return m_pidOutput;
	}

	@Override public double getSetpoint() {
		return m_setpoint;
	}

	@Override public double getError() {
		double e;
		synchronized (this) { e = getSetpoint() - m_pidInput.pidGet(); }
		return getContinuousError(e);
	}

	@Override public double[] getInputRange() {
		double[] out;
		synchronized (this) {
			out = new double[] {m_minimumInput, m_maximumInput};
		}
		return out;
	}

	@Override public double[] getOutputRange() {
		double[] out;
		synchronized (this) {
			out = new double[] {m_minimumOutput, m_maximumOutput};
		}
		return out;
	}

	@Override public Tolerance getTolerance() {
		return m_tolerance;
	}

	@Override public boolean isContinuous() {
		return m_continuous;
	}

	@Override public double get() {
		return m_result;
	}



	//===================//
	//      Setters      //
	//===================//
	
	@Override public void setPeriod(long period) {
		if (period <= 0) {throw new IllegalArgumentException();}
		synchronized (this) { m_period = period; }
	}

	@Override public void setP(double kP) {
		if (!Double.isFinite(kP)) {throw new IllegalArgumentException();}
		synchronized (this) { m_P = kP; }
	}

	@Override public void setI(double kI) {
		if (!Double.isFinite(kI)) {throw new IllegalArgumentException();}
		synchronized (this) { m_I = kI; }
	}
	
	@Override public void setD(double kD) {
		if (!Double.isFinite(kD)) {throw new IllegalArgumentException();}
		synchronized (this) { m_D = kD; }
	}
	
	@Override public void setF(double kF) {
		if (!Double.isFinite(kF)) {throw new IllegalArgumentException();}
		synchronized (this) { m_F = kF; }
	}

	@Override public void setIZone(double iZone) {
		if (!Double.isFinite(iZone) || iZone <= 0) {throw new IllegalArgumentException();}
		synchronized (this) { m_iZone = iZone; }
	}
	
	@Override public void setSource(PIDSource source) {
		if (source == null) {throw new NullPointerException();}
		synchronized (this) { m_pidInput = source; }
	}

	@Override public void setOutput(PIDOutput output) {
		if (output == null) {throw new NullPointerException();}
		synchronized (this) { m_pidOutput = output; }
	}
	
	@Override public double setSetpoint(double setpoint) {
		if (!Double.isFinite(setpoint)) {throw new IllegalArgumentException();}
		synchronized (this) {
			setpoint = InterfacePID.clamp(setpoint, m_minimumInput, m_maximumInput);
			m_setpoint = setpoint;
		}
		return setpoint;
	}
	
	@Override public void setInputRange(double minimumInput, double maximumInput) {
		if (Double.isNaN(minimumInput) || Double.isNaN(maximumInput) || minimumInput > maximumInput) {throw new IllegalArgumentException();}
		synchronized (this) {
			m_minimumInput = minimumInput;
			m_maximumInput = maximumInput;
			setSetpoint(m_setpoint);
		}
	}

	@Override public void setOutputRange(double minimumOutput, double maximumOutput) {
		if (Double.isNaN(minimumOutput) || Double.isNaN(maximumOutput) || minimumOutput > maximumOutput) {throw new IllegalArgumentException();}
		synchronized (this) {
			m_minimumOutput = minimumOutput;
			m_maximumOutput = maximumOutput;
		}
	}
	
	@Override public void setTolerance(Tolerance tol) {
		m_tolerance = tol;
	}

	@Override public void setContinuous(boolean continuous) {
		m_continuous = continuous;
	}



	//============================//
	//      Override Methods      //
	//============================//
	
	@Override public void initSendable(SendableBuilder builder) {
		builder.setSmartDashboardType("PIDBase");
		builder.setSafeState(this::reset);
		builder.addDoubleProperty("p", this::getP, this::setP);
		builder.addDoubleProperty("i", this::getI, this::setI);
		builder.addDoubleProperty("d", this::getD, this::setD);
		builder.addDoubleProperty("f", this::getF, this::setF);
		builder.addDoubleProperty("setpoint", this::getSetpoint, this::setSetpoint);
		builder.addBooleanProperty("enabled", this::isEnabled, this::setEnabled);
	}
	
	// @Override public void free() {
	// 	super.free();
	// 	synchronized (this) {
	// 		m_pidInput = null;
	// 		m_pidOutput = null;
	// 	}
	// }
	
	/**
	 * Disables the PID and resets all accumulated and previous error to zero. Additionally,
	 * the result is also set to zero.
	 */
	public void reset() {
		synchronized (this) {
			setEnabled(false);
			m_prevInput = 0.0;
			m_accumError = 0.0;
			m_result = 0.0;
		}
	}



	//==================================//
	//      Private Helper Methods      //
	//==================================//

	/**
	 * Wraps error around for continuous inputs. The original error is returned if continuous mode is
	 * disabled. This is an unsynchronized function.
	 *
	 * @param error The current error
	 * @return the resulting error
	 */
	private double getContinuousError(double error) {
		double[] irange = getInputRange();
		double range = irange[1] - irange[0];
		if (isContinuous() && Double.isFinite(range)) {
			error %= range;
			if (Math.abs(error) > range / 2) {
				if (error > 0) {
					return error - range;
				} else {
					return error + range;
				}
			}
		}
		return error;
	}

}
