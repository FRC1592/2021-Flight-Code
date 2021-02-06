/**
 * 
 */
package frc.robot.lib1592.drivers;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;

import frc.robot.lib1592.control.PIDConstantsCTRE;
import frc.robot.lib1592.drivers.TalonSRX.Slot;

/**
 * Talon Builder Factory
 * <p>
 * Includes an internal configuration class for configuring a Talon on creation as well as
 * a number of helper creation methods.
 */
public class TalonFactory {
	
	//===========================//
	//    Configuration Class    //
	//===========================//
	
	/**
	 * Talon Builder Configuration
	 * <p>
	 * Base class for using as an input for the creation of a Talon. Any null values are treated as no input
	 * and those commands are not set on the Talon. Some members have additional restrictions which prevent
	 * settings from being applied to the Talon.
	 */
	public static class Configuration {
		public Boolean safetyEnabled = null;			// Safety Enabled (SPECIAL CASE: If not true, safetyExpiration is ignored)
		public Double safetyExpiration = null;			// Motor Safety Timeout [s] (Non-Finite or negative values are treated as no input)
		
		public Boolean sensorPhase = null;				// Talon Sensor Phase
		public Boolean inverted = null;					// Talon Inversion
		
		public final Map<ControlFrame, Integer> controlFramePeriodMap = new HashMap<>();			// Control Frame Period Map (Period in ms; Negative or zero values are treated as no input; Null key is ignored)
		public final Map<StatusFrameEnhanced, Integer> statusFramePeriodMap = new HashMap<>();		// Status Frame Period Map (Period in ms; Negative or zero values are treated as no input; Null key is ignored)
		
		public Integer currentLimitContinuous = null;		// Continuous Current Limit [amps] (Negative values are treated as no input)
		public Integer currentLimitPeak = null;				// Peak Current Limit [amps] (Negative values are treated as no input)
		public Integer currentLimitPeakDuration = null;		// Duration Of Peak Current Limit [ms] (Negative values are treated as no input)
		
		public Integer voltageSampleWindow = null;				// Voltage Sample Window For Rolling Average (Negative or zero values are treated as no input)
		public Double voltageCompensationSaturation = null;		// Voltage Compensation Saturation [volts] (Non-Finite, negative, or zero values are treated as no input)
		public Boolean enableVoltageCompensation = null;
		
		public Boolean forwardSoftLimitEnabled = null;		// Forward Sensor Limit Enabled
		public Integer forwardSoftLimitThreshold = null;	// Forward Sensor Limit Threshold [Sensor Units]
		public Boolean reverseSoftLimitEnabled = null;		// Reverse Sensor Limit Enabled
		public Integer reverseSoftLimitThreshold = null;	// Reverse Sensor Limit Threshold [Sensor Units]
		
		public Double forwardNominalOutput = null;		// Forward Nominal (min) Output (Must be [0,1])
		public Double forwardPeakOutput = null;			// Forward Peak (max) Output (Must be [0,1])
		public Double reverseNominalOutput = null;		// Reverse Nominal (min) Output (Must be [0,1])
		public Double reversePeakOutput = null;			// Reverse Peak (max) Output (Must be [0,1])
		
		public VelocityMeasPeriod velocityMeasurementPeriod = null;		// Velocity Measurement Period
		public Integer velocitySampleWindow = null;						// Velocity Sample Window For Rolling Average (Negative or zero values are treated as no input)
		
		public FeedbackDevice feedbackDevice = null;			// Feedback Sensor Map (Null key is ignored)
		public final Map<Slot, PIDConstantsCTRE> pidMap = new HashMap<>();					// Closed Loop Constants Map (Null key is ignored)
		
		public Integer configTimeout = null; 			// Configuration Setter Timeout Value [ms] (Negative or zero values are treated as if no timeout is set)
		
		public NeutralMode neutralMode = null;
		
		
		
		//====================//
		//    Constructors    //
		//====================//
		
		/** Private Constructor : Empty Configuration */
		private Configuration() {};
		
		/** Empty Configuration Constructor */
		public static Configuration empty() {return new Configuration();}
		
		/** Standard Configuration Constructor */
		public static Configuration standard() {
			Configuration config = empty();
			config.reverseSoftLimitEnabled = false;
			config.forwardSoftLimitEnabled = false;
			config.voltageCompensationSaturation = 12d;
			config.enableVoltageCompensation = false;
			return config;
		}
		
		/** Slave Standard Configuration Constructor */
		public static Configuration standardSlave() {
			Configuration config = standard();
			//Turn down status frames to cut down bus traffic
			config.statusFramePeriodMap.put(StatusFrameEnhanced.Status_1_General, 1000);
			config.statusFramePeriodMap.put(StatusFrameEnhanced.Status_2_Feedback0, 1000);
			config.statusFramePeriodMap.put(StatusFrameEnhanced.Status_3_Quadrature, 1000);
			config.statusFramePeriodMap.put(StatusFrameEnhanced.Status_4_AinTempVbat, 1000);
			config.statusFramePeriodMap.put(StatusFrameEnhanced.Status_8_PulseWidth, 1000);
			return config;
		}
		
		/** Copy Constructor */
		public Configuration copy() {
			Configuration c = Configuration.empty();
			c.safetyEnabled = safetyEnabled;
			c.safetyExpiration = safetyExpiration;
			c.sensorPhase = sensorPhase;
			c.inverted = inverted;
			c.controlFramePeriodMap.putAll(controlFramePeriodMap);
			c.statusFramePeriodMap.putAll(statusFramePeriodMap);
			c.currentLimitContinuous = currentLimitContinuous;
			c.currentLimitPeak = currentLimitPeak;
			c.currentLimitPeakDuration = currentLimitPeakDuration;
			c.voltageSampleWindow = voltageSampleWindow;
			c.voltageCompensationSaturation = voltageCompensationSaturation;
			c.enableVoltageCompensation = enableVoltageCompensation;
			c.forwardSoftLimitEnabled = forwardSoftLimitEnabled;
			c.forwardSoftLimitThreshold = forwardSoftLimitThreshold;
			c.reverseSoftLimitEnabled = reverseSoftLimitEnabled;
			c.reverseSoftLimitThreshold = reverseSoftLimitThreshold;
			c.forwardNominalOutput = forwardNominalOutput;
			c.forwardPeakOutput = forwardPeakOutput;
			c.reverseNominalOutput = reverseNominalOutput;
			c.reversePeakOutput = reversePeakOutput;
			c.velocityMeasurementPeriod = velocityMeasurementPeriod;
			c.velocitySampleWindow = velocitySampleWindow;
			c.feedbackDevice = feedbackDevice;
			c.pidMap.putAll(pidMap);
			c.configTimeout = configTimeout;
			c.neutralMode = neutralMode;
			return c;
		}
	}
	
	
	
	//=================================//
	//    Static Talon Constructors    //
	//=================================//
	
	/**
	 * Create a Talon using the standard configuration
	 *
	 * @param id  the talon id
	 * @return the created Talon
	 */
	public static TalonSRX create(int id) {
		return create(id, Configuration.standard());
	}
	
	/**
	 * Create a Talon using the input configuration
	 * 
	 * @param id  the talon id
	 * @param config  the talon configuration object
	 * @return the created Talon
	 */
	public static TalonSRX create(int id, Configuration config) {
		if (config==null) {throw new NullPointerException();}
		
		TalonSRX talon = new TalonSRX(id);
		
		int timeout = (config.configTimeout == null || config.configTimeout < 0) ? 0 : config.configTimeout;
		
		if (config.safetyEnabled !=null && config.safetyEnabled && config.safetyExpiration != null && Double.isFinite(config.safetyExpiration) && config.safetyExpiration > 0) { talon.setExpiration(config.safetyExpiration); }
		if (config.safetyEnabled != null) { talon.setSafetyEnabled(config.safetyEnabled); }
		
		if (config.sensorPhase != null) { talon.setSensorPhase(config.sensorPhase); }
		if (config.inverted != null) { talon.setInverted(config.inverted); }
		
		for (Map.Entry<ControlFrame, Integer> e : config.controlFramePeriodMap.entrySet()) {
			if (e.getKey() != null && e.getValue() != null && e.getValue() >= 0) { talon.setControlFramePeriod(e.getKey(), e.getValue()); }
		}
		for (Map.Entry<StatusFrameEnhanced, Integer> e : config.statusFramePeriodMap.entrySet()) {
			if (e.getKey() != null && e.getValue() != null && e.getValue() >= 0) { talon.setStatusFramePeriod(e.getKey(), e.getValue(), timeout); }
		}
		
		if (config.currentLimitContinuous != null && config.currentLimitContinuous >= 0) { talon.configContinuousCurrentLimit(config.currentLimitContinuous, timeout); }
		if (config.currentLimitPeak != null && config.currentLimitPeak >= 0) { talon.configPeakCurrentLimit(config.currentLimitPeak, timeout); }
		if (config.currentLimitPeakDuration != null && config.currentLimitPeakDuration >= 0) { talon.configPeakCurrentDuration(config.currentLimitPeakDuration, timeout); }
		
		if (config.voltageSampleWindow != null && config.voltageSampleWindow > 0) { talon.configVoltageMeasurementFilter(config.voltageSampleWindow, timeout); }
		if (config.voltageCompensationSaturation != null && Double.isFinite(config.voltageCompensationSaturation) && config.voltageCompensationSaturation > 0) { talon.configVoltageCompSaturation(config.voltageCompensationSaturation, timeout); }
		
		if (config.forwardSoftLimitEnabled != null) { talon.configForwardSoftLimitEnable(config.forwardSoftLimitEnabled, timeout); }
		if (config.forwardSoftLimitThreshold != null) { talon.configForwardSoftLimitThreshold(config.forwardSoftLimitThreshold, timeout); }
		if (config.reverseSoftLimitEnabled != null) { talon.configReverseSoftLimitEnable(config.reverseSoftLimitEnabled, timeout); }
		if (config.reverseSoftLimitThreshold != null) { talon.configReverseSoftLimitThreshold(config.reverseSoftLimitThreshold, timeout); }
		
		if (validRatio(config.forwardNominalOutput)) { talon.configNominalOutputForward(config.forwardNominalOutput, timeout); }
		if (validRatio(config.forwardPeakOutput)) { talon.configPeakOutputForward(config.forwardPeakOutput, timeout); }
		if (validRatio(config.reverseNominalOutput)) { talon.configNominalOutputReverse(config.reverseNominalOutput, timeout); }
		if (validRatio(config.reversePeakOutput)) { talon.configPeakOutputReverse(config.reversePeakOutput, timeout); }
		
		if (config.velocityMeasurementPeriod != null) { talon.configVelocityMeasurementPeriod(config.velocityMeasurementPeriod, timeout); }
		if (config.velocitySampleWindow != null && config.velocitySampleWindow > 0) { talon.configVelocityMeasurementWindow(config.velocitySampleWindow, timeout); }
		
		if (config.feedbackDevice != null) { talon.configSelectedFeedbackSensor(config.feedbackDevice,0,timeout); }
		
		for (Map.Entry<Slot, PIDConstantsCTRE> e : config.pidMap.entrySet()) {
			if (e.getKey() != null && e.getValue() != null) { talon.setPID(e.getKey(), e.getValue()); }
		}
		
		if (config.neutralMode != null) { talon.setNeutralMode(config.neutralMode); }
		
		return talon;
	}
	
	/**
	 * Create a slave Talon using the standard slave configuration
	 *
	 * @param id  the talon id
	 * @param mid  the talon id of the master
	 * @return the created Talon
	 */
	public static TalonSRX createSlave(int id, int mid) {
		return createSlave(id, mid, Configuration.standardSlave());
	}
	
	/**
	 * Create a slave Talon using the input configuration
	 *
	 * @param id  the talon id
	 * @param mid  the talon id of the master
	 * @param config  the talon configuration object
	 * @return the created Talon
	 */
	public static TalonSRX createSlave(int id, int mid, Configuration config) {
		TalonSRX out = create(id, config);
		out.set(ControlMode.Follower, mid);
		return out;
	}
	
	
	
	//============================//
	//    Private Static Class    //
	//============================//

	/**
	 * Internal Lazy Talon Implementation
	 */
	@SuppressWarnings("unused")
	private static class LazyTalonSRX extends TalonSRX {
		private long expiration;
		private double lastSet = Double.NaN;
		private ControlMode lastMode = null;
		private long root;
		
		/**
		 * Standard Lazy Talon Constructor
		 *
		 * @param id  motor controller id
		 */
		public LazyTalonSRX(int id, Double safetyExpiration) {
			super(id);
			expiration = (long) (0.75 * 1e9 * ((safetyExpiration == null || !Double.isFinite(safetyExpiration)  || safetyExpiration <= 0) ? 0 : safetyExpiration));
			root = System.nanoTime();
		}
		
		@Override public void set(double value) {
			long elapsed = System.nanoTime()-root;
	        if (value != lastSet || getControlMode() != lastMode || (expiration > 0 && elapsed > expiration)) {
	        	lastSet = value;
	        	lastMode = getControlMode();
	            super.set(value);
	        }
	    }
		
	}
	
	
	
	//==============================//
	//    Private Static Methods    //
	//==============================//
	
	/**
	 * Internal Helper Method
	 */
	private static boolean validRatio(Double value) {
		return value != null && Double.isFinite(value) && value>= 0 && value <= 1;
	}
	
}
