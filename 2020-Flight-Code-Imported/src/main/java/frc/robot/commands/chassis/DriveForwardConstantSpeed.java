// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.DeadReckoning;
import frc.robot.subsystems.Chassis;

public class DriveForwardConstantSpeed extends CommandBase {
  private final Chassis m_chassis;
  private final double m_speed;

  private final DeadReckoning m_deadReckoning;

  /** Creates a new DriveFowardForTime. */
  public DriveForwardConstantSpeed(Chassis chassis, double speed) {
    m_chassis = chassis;
    m_speed = speed;

    m_deadReckoning = new DeadReckoning();

    addRequirements(m_chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_deadReckoning.startTimer();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_chassis.drive(m_speed, m_speed);

    // Setpoint is implicitly 0, since we don't want the heading to change
    double error = -m_chassis.getRate();

    // Drives forward continuously at half speed, using the gyro to stabilize the heading
    m_chassis.drive(m_speed + Constants.GAIN_CHASSIS_DRIVE_FORWARD.kP * error, m_speed - Constants.GAIN_CHASSIS_DRIVE_FORWARD.kP * error);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_deadReckoning.stopTimer();
    m_deadReckoning.printDriveForwardCmd();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

// Example: Tank drive stabilization using turn rate
// https://docs.wpilib.org/en/stable/docs/software/sensors/gyros-software.html?highlight=gyro#example-tank-drive-stabilization-using-turn-rate
// When closing the loop on the turn rate for heading stabilization, PI loops are particularly effective.