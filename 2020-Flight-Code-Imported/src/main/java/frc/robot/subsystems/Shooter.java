/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib1592.drivers.TalonFactory;
import frc.robot.lib1592.drivers.TalonSRX;
import frc.robot.lib1592.hids.XBoxGamepad;

public class Shooter extends SubsystemBase {
  private final TalonSRX m_gather = TalonFactory.create(Constants.ID_GATHER);
  private final TalonSRX m_kicker = TalonFactory.create(Constants.ID_KICKER);
  private final TalonFX m_shooter = new TalonFX(Constants.ID_SHOOTER);
  private final DigitalInput m_loaded = new DigitalInput(Constants.DIO_LOADED);

  /* String for output */
  StringBuilder _sb = new StringBuilder();
    
  /* Loop tracker for prints */
  int _loops = 0;

  public Shooter(XBoxGamepad joyDriver) {
    m_gather.setInverted(Constants.INVERT_GATHER);
    m_kicker.setInverted(Constants.INVERT_KICKER);
    m_shooter.setInverted(Constants.INVERT_SHOOTER);

    /* Factory Default all hardware to prevent unexpected behaviour */
		m_shooter.configFactoryDefault();
		
		/* Config neutral deadband to be the smallest possible */
		m_shooter.configNeutralDeadband(0.001);
    
		/* Config sensor used for Primary PID [Velocity] */
    m_shooter.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.PID_LOOP_IDX,	Constants.TALON_FX_TIMEOUT);

		/* Config the peak and nominal outputs */
		m_shooter.configNominalOutputForward(0, Constants.TALON_FX_TIMEOUT);
		m_shooter.configNominalOutputReverse(0, Constants.TALON_FX_TIMEOUT);
		m_shooter.configPeakOutputForward(1, Constants.TALON_FX_TIMEOUT);
		m_shooter.configPeakOutputReverse(-1, Constants.TALON_FX_TIMEOUT);

		/* Config the Velocity closed loop gains in slot0 */
		m_shooter.config_kF(Constants.PID_LOOP_IDX, Constants.GAIN_VELOCITY.kF, Constants.TALON_FX_TIMEOUT);
		m_shooter.config_kP(Constants.PID_LOOP_IDX, Constants.GAIN_VELOCITY.kP, Constants.TALON_FX_TIMEOUT);
		m_shooter.config_kI(Constants.PID_LOOP_IDX, Constants.GAIN_VELOCITY.kI, Constants.TALON_FX_TIMEOUT);
    m_shooter.config_kD(Constants.PID_LOOP_IDX, Constants.GAIN_VELOCITY.kD, Constants.TALON_FX_TIMEOUT);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Limit Switch", isLoaded());

    /* Get Talon/Victor's current RPM */
    _sb.append("\trpm:");
    _sb.append(m_shooter.getSelectedSensorVelocity(Constants.PID_LOOP_IDX) / Constants.RPM_TO_UNITS_PER_100MS);
    _sb.append("rpm"); 	// RPM

    /* Get Talon/Victor's current output percentage */
		double motorOutput = m_shooter.getMotorOutputPercent();
		
		/* Prepare line to print */
    _sb.append("\tout:");
    
		/* Cast to int to remove decimal places */
		_sb.append((int) (motorOutput * 100));
		_sb.append("%");	// Percent

		_sb.append("\tspd:");
		_sb.append(m_shooter.getSelectedSensorVelocity(Constants.PID_LOOP_IDX));
    _sb.append("u"); 	// Native units
     
/* Append more signals to print when in speed mode. */
_sb.append("\terr:");
_sb.append(m_shooter.getClosedLoopError(Constants.PID_LOOP_IDX));
_sb.append("\ttrg:");
_sb.append(Constants.TARGET_VELOCITY_UNITS_PER_100_MS);
    
    /* Print built string every 10 loops */
		if (++_loops >= 10) {
      _loops = 0;
      System.out.println(_sb.toString());
    }

    /* Reset built string */
		_sb.setLength(0);
  }
  
  public void startGather() {
    m_gather.set(ControlMode.PercentOutput, Constants.SPEED_GATHER);
    m_kicker.set(ControlMode.PercentOutput, -0.3);
  }
  
  public void reverseGather() {
    m_gather.set(ControlMode.PercentOutput, -Constants.SPEED_GATHER);
  }
  
  public void stopGather() {
    m_gather.stopMotor();
    m_kicker.stopMotor();
  }
  
  public void startKicker() {
    m_gather.set(ControlMode.PercentOutput, Constants.SPEED_GATHER);
    m_kicker.set(ControlMode.PercentOutput, Constants.SPEED_KICKER);
  }
  
  public void stopKicker() {
    m_gather.stopMotor();
    m_kicker.stopMotor();
  }
  
  public void startShooter() {
    m_shooter.set(TalonFXControlMode.Velocity, Constants.TARGET_VELOCITY_UNITS_PER_100_MS);
  }

  public void stopShooter() {
    m_shooter.set(ControlMode.PercentOutput, 0.0);
  }

  public boolean isLoaded() {
    return m_loaded.get() ^ Constants.INVERT_LOADED;
  }
}
