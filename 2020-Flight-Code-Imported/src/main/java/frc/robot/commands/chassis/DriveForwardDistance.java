// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Chassis;

public class DriveForwardDistance extends CommandBase {
  private final Chassis m_chassis;
  private final double m_distanceMeters;
  private double m_startDistanceMeters;

  /** Creates a new DriveForwardDistance. */
  public DriveForwardDistance(Chassis chassis, double distanceMeters) {
    m_chassis = chassis;
    m_distanceMeters = distanceMeters;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startDistanceMeters = m_chassis.getDistanceMeters();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_chassis.drive(Constants.AUTO_SPEED_FORWARD, Constants.AUTO_SPEED_FORWARD);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_chassis.getDistanceMeters() - m_startDistanceMeters > m_distanceMeters;
  }
}
