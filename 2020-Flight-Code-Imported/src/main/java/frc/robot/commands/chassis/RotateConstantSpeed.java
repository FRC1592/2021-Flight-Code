// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class RotateConstantSpeed extends CommandBase {
  private final Chassis m_chassis;
  private final double m_leftSpeed;
  private final double m_rightSpeed;

  private double startTime;
  private double elapsedTime = 0.0;

  /** Creates a new DriveFowardForTime. */
  public RotateConstantSpeed(Chassis chassis, double leftSpeed, double rightSpeed) {
    m_chassis = chassis;
    m_leftSpeed = -leftSpeed;
    m_rightSpeed = -rightSpeed;
    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set start time.
    startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_chassis.drive(m_leftSpeed, m_rightSpeed); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elapsedTime = System.currentTimeMillis() - startTime;

    if (m_leftSpeed > 0.0) {
      System.out.println("new RotateConstantSpeed(m_chassis, -Constants.AUTO_SPEED_ROTATE, Constants.AUTO_SPEED_ROTATE).withTimeout(" + elapsedTime + " / 1000.0),");
      System.out.println("new WaitCommand(1.0),");
    } else {
      System.out.println("new RotateConstantSpeed(m_chassis, Constants.AUTO_SPEED_ROTATE, -Constants.AUTO_SPEED_ROTATE).withTimeout(" + elapsedTime + " / 1000.0),");
      System.out.println("new WaitCommand(1.0),");
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
