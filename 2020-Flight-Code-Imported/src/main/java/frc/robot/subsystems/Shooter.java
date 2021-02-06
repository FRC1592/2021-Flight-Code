/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib1592.drivers.TalonFactory;
import frc.robot.lib1592.drivers.TalonSRX;
import frc.robot.lib1592.drivers.TalonSRX.Slot;

public class Shooter extends SubsystemBase {
  private final TalonSRX m_gather = TalonFactory.create(Constants.ID_GATHER);
  private final TalonSRX m_kicker = TalonFactory.create(Constants.ID_KICKER);
  private final TalonSRX m_shooter = TalonFactory.create(Constants.ID_SHOOTER);
  private final DigitalInput m_loaded = new DigitalInput(Constants.DIO_LOADED);

  public Shooter() {
    m_gather.setInverted(Constants.INVERT_GATHER);
    m_kicker.setInverted(Constants.INVERT_KICKER);
    m_shooter.setInverted(Constants.INVERT_SHOOTER);
    m_shooter.setSensorPhase(Constants.INVERT_SHOOTER_SENSOR);

    m_shooter.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.TALON_TIMEOUT);
    m_shooter.configAllowableClosedloopError(0, Constants.ERROR_MAX_SHOOTER * Constants.ENC_SHOOTER_PPR, Constants.TALON_TIMEOUT);
    m_shooter.setPID(Slot.SLOT0, Constants.PID_SHOOTER);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Limit Switch", isLoaded());
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
    m_shooter.set(ControlMode.Velocity, Constants.SPEED_SHOOTER * Constants.ENC_SHOOTER_PPR * Constants.MIN_TO_100ms);
  }

  public void stopShooter() {
    m_shooter.stopMotor();
  }

  public boolean isLoaded() {
    return m_loaded.get() ^ Constants.INVERT_LOADED;
  }
}
