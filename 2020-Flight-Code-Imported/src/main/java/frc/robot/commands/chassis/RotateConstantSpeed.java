// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;
import frc.robot.DeadReckoning;

public class RotateConstantSpeed extends CommandBase {
  private final Chassis m_chassis;
  private final double m_leftSpeed;
  private final double m_rightSpeed;

  private final DeadReckoning m_deadReckoning;
  private final String m_direction;

  /** Creates a new DriveFowardForTime. */
  public RotateConstantSpeed(Chassis chassis, double leftSpeed, double rightSpeed, String direction) {
    m_chassis = chassis;
    m_leftSpeed = leftSpeed;
    m_rightSpeed = rightSpeed;

    m_deadReckoning = new DeadReckoning();
    m_direction = direction;

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
    m_chassis.drive(m_leftSpeed, m_rightSpeed); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_deadReckoning.stopTimer();
    m_deadReckoning.printRotateConstantSpeedCmd(m_direction);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
