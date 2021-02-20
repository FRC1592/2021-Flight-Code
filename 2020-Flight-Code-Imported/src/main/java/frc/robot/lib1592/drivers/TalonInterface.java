package frc.robot.lib1592.drivers;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.StickyFaults;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;

import frc.robot.lib1592.control.PIDConstantsCTRE;
import frc.robot.lib1592.drivers.TalonSRX.Slot;

import edu.wpi.first.wpilibj.Sendable;

public interface TalonInterface {
	
	public static interface UnmodifiableTalonSRX {
		public double get();
		// public double getActiveTrajectoryHeading();
		// public int getActiveTrajectoryPosition();
		// public int getActiveTrajectoryVelocity();
		public int getBaseID();
		public double getBusVoltage();
		// public int getClosedLoopError(int pidIdx);
		public ControlMode getControlMode();
		public double getD(Slot slot);
		public String getDescription();
		public int getDeviceID();
		public double getErrorDerivative(int pidIdx);
		// public int getClosedLoopTarget(int pidIdx);
		double getExpiration();
		public double getF(Slot slot);
		public ErrorCode getFaults(Faults toFill);
		public int getFirmwareVersion();
		public long getHandle();
		public double getI(Slot slot);
		public double getIntegralAccumulator(int pidIdx);
		public boolean getInverted();
		public int getIZone(Slot slot);
		public ErrorCode getLastError();
		public ErrorCode getMotionProfileStatus(MotionProfileStatus statusToFill);
		public int getMotionProfileTopLevelBufferCount();
		public double getMotorOutputPercent();
		public double getMotorOutputVoltage();
		// public String getName();
		public double getOutputCurrent();
		public double getP(Slot slot);
		public PIDConstantsCTRE getPID(Slot slot);
		// public int getSelectedSensorPosition(int pidIdx);
		// public int getSelectedSensorVelocity(int pidIdx);
		public int getStatusFramePeriod(StatusFrame frame, int timeoutMs);
		public int getStatusFramePeriod(StatusFrameEnhanced frame, int timeoutMs);
		public int getStatusFramePeriod(int frame, int timeoutMs);
		public ErrorCode getStickyFaults(StickyFaults toFill);
		// public String getSubsystem();
		public double getTemperature();
		public boolean hasResetOccurred();
		public boolean isAlive();
		public boolean isMotionProfileTopLevelBufferFull();
		public boolean isSafetyEnabled();
	}
	
	public static interface Navigation {
		public ErrorCode changeMotionControlFramePeriod(int periodMs);
		public ErrorCode clearMotionProfileHasUnderrun(int timeoutMs);
		public ErrorCode clearMotionProfileTrajectories();
		// public ErrorCode configMotionAcceleration(int sensorUnitsPer100msPerSec, int timeoutMs);
		// public ErrorCode configMotionCruiseVelocity(int sensorUnitsPer100ms, int timeoutMs);
		public void processMotionProfileBuffer();
		// public ErrorCode pushMotionProfileTrajectory(TrajectoryPoint trajPt);
		// public void enableHeadingHold(boolean enable);
	}
	
	public static interface ClosedLoop {
		// public ErrorCode config_IntegralZone(int slotIdx, int izone, int timeoutMs);
		public ErrorCode config_kD(int slotIdx, double value, int timeoutMs);
		public ErrorCode config_kF(int slotIdx, double value, int timeoutMs);
		public ErrorCode config_kI(int slotIdx, double value, int timeoutMs);
		public ErrorCode config_kP(int slotIdx, double value, int timeoutMs);
		// public ErrorCode configAllowableClosedloopError(int slotIdx, int allowableClosedLoopError, int timeoutMs);
		public ErrorCode configClosedloopRamp(double secondsFromNeutralToFull, int timeoutMs);
		public ErrorCode configMaxIntegralAccumulator(int slotIdx, double iaccum, int timeoutMs);
		// public void selectDemandType(boolean value);
		public void selectProfileSlot(int slotIdx, int pidIdx);
		public void setD(Slot slot, double d);
		public void setF(Slot slot, double d);
		public void setI(Slot slot, double d);
		public void setIZone(Slot slot, int d);
		public void setP(Slot slot, double d);
		public void setPID(Slot slot, PIDConstantsCTRE consts);
		public ErrorCode setIntegralAccumulator(double iaccum, int pidIdx, int timeoutMs);
	}
	
	public static interface Sensor {
		public ErrorCode configRemoteFeedbackFilter(int deviceID, RemoteSensorSource remoteSensorSource, int remoteOrdinal, int timeoutMs);
		public ErrorCode configSelectedFeedbackSensor(FeedbackDevice feedbackDevice, int pidIdx, int timeoutMs);
		public ErrorCode configSelectedFeedbackSensor(RemoteFeedbackDevice feedbackDevice, int pidIdx, int timeoutMs);
		public ErrorCode configSensorTerm(SensorTerm sensorTerm, FeedbackDevice feedbackDevice, int timeoutMs);
		public SensorCollection getSensorCollection();
		// public ErrorCode setSelectedSensorPosition(int sensorPos, int pidIdx, int timeoutMs);
		public void setSensorPhase(boolean PhaseSensor);
	}
	
	public static interface Hardware {
		public ErrorCode clearStickyFaults(int timeoutMs);
		public ErrorCode configContinuousCurrentLimit(int amps, int timeoutMs);
		public ErrorCode configForwardLimitSwitchSource(LimitSwitchSource type, LimitSwitchNormal normalOpenOrClose, int timeoutMs);
		public ErrorCode configForwardLimitSwitchSource(RemoteLimitSwitchSource type, LimitSwitchNormal normalOpenOrClose, int deviceID, int timeoutMs);
		public ErrorCode configForwardSoftLimitEnable(boolean enable, int timeoutMs);
		// public ErrorCode configForwardSoftLimitThreshold(int forwardSensorLimit, int timeoutMs);
		public ErrorCode configNeutralDeadband(double percentDeadband, int timeoutMs);
		public ErrorCode configNominalOutputForward(double percentOut, int timeoutMs);
		public ErrorCode configNominalOutputReverse(double percentOut, int timeoutMs);
		public ErrorCode configOpenloopRamp(double secondsFromNeutralToFull, int timeoutMs);
		public ErrorCode configPeakCurrentDuration(int milliseconds, int timeoutMs );
		public ErrorCode configPeakCurrentLimit(int amps, int timeoutMs );
		public ErrorCode configPeakOutputForward(double percentOut, int timeoutMs);
		public ErrorCode configPeakOutputReverse(double percentOut, int timeoutMs);
		public ErrorCode configReverseLimitSwitchSource(LimitSwitchSource type, LimitSwitchNormal normalOpenOrClose, int timeoutMs);
		public ErrorCode configReverseSoftLimitEnable(boolean enable, int timeoutMs);
		// public ErrorCode configReverseSoftLimitThreshold(int forwardSensorLimit, int timeoutMs);
		public ErrorCode configVelocityMeasurementPeriod(VelocityMeasPeriod period, int timeoutMs);
		public ErrorCode configVelocityMeasurementWindow(int windowSize, int timeoutMs);
		public ErrorCode configVoltageCompSaturation(double voltage, int timeoutMs);
		public ErrorCode configVoltageMeasurementFilter(int filterWindowSamples, int timeoutMs);
		public void enableCurrentLimit(boolean enable);
		public void enableVoltageCompensation(boolean enable);
		public void setInverted(boolean invert);
		public void setNeutralMode(NeutralMode neutralMode);
		public void overrideLimitSwitchesEnable(boolean enable);
		public void overrideSoftLimitsEnable(boolean enable);
		public void valueUpdated();
	}
	
	public static interface Safety {
		public void setExpiration(double timeout);
		public void setSafetyEnabled(boolean enabled);
	}
	
	public static interface Parameters {
		public int configGetCustomParam(int paramIndex, int timoutMs);
		public double configGetParameter(ParamEnum param, int ordinal, int timeoutMs);
		public double configGetParameter(int param, int ordinal, int timeoutMs);
		public ErrorCode configSetCustomParam(int newValue, int paramIndex, int timeoutMs);
		public ErrorCode configSetParameter(ParamEnum param, double value, int subValue, int ordinal, int timeoutMs);
		public ErrorCode configSetParameter(int param, double value, int subValue, int ordinal, int timeoutMs);
		public ErrorCode setControlFramePeriod(ControlFrame frame, int periodMs);
		public ErrorCode setControlFramePeriod(int frame, int periodMs);
		public ErrorCode setStatusFramePeriod(StatusFrame frame, int periodMs, int timeoutMs);
		public ErrorCode setStatusFramePeriod(StatusFrameEnhanced frame, int periodMs, int timeoutMs);
		public ErrorCode setStatusFramePeriod(int frameValue, int periodMs, int timeoutMs);
	}
	
	public static interface WPI {
		public void free();
	}
	
	public static interface ProtectedTalonSRX extends UnmodifiableTalonSRX, Navigation, ClosedLoop, Sensor, Hardware, Safety, Parameters, WPI, Sendable {}
	
	public static interface Action {
		public void disable();
		public void follow(IMotorController masterToFollow);
		public void neutralOutput();
		public void pidWrite(double output);
		public void set(ControlMode Mode, double demand);
		// public void set(ControlMode Mode, double demand0, double demand1);
		public void set(double speed);
		public void stopMotor();
	}
	
}
