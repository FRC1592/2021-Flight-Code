// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class DriveFowardConstantSpeed extends CommandBase {
  private final Chassis m_chassis;
  private final double m_speed;

  private double startTime;
  private double elapsedTime = 0.0;

  /** Creates a new DriveFowardForTime. */
  public DriveFowardConstantSpeed(Chassis chassis, double speed) {
    m_chassis = chassis;
    m_speed = -speed;
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
    m_chassis.drive(m_speed, m_speed); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elapsedTime = System.currentTimeMillis() - startTime;
    System.out.println("new DriveFowardConstantSpeed(m_chassis, Constants.AUTO_SPEED_FORWARD).withTimeout(" + elapsedTime + " / 1000.0),");
    System.out.println("new WaitCommand(1.0),");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
