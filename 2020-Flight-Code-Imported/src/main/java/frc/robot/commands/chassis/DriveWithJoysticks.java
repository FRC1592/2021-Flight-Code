/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.chassis;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class DriveWithJoysticks extends CommandBase {
  private final Chassis m_chassis;
  private final DoubleSupplier m_left;
  private final DoubleSupplier m_right;

  public DriveWithJoysticks(Chassis chassis, DoubleSupplier left, DoubleSupplier right) {
    m_chassis = chassis;
    m_left = left;
    m_right = right;
    addRequirements(m_chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  /**
   * Called every time the scheduler runs while the command is scheduled.
   * Polls the XboxController and passes values to Chassis' drive() method.
   */
  @Override
  public void execute() {
    m_chassis.drive(m_left.getAsDouble(), m_right.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
