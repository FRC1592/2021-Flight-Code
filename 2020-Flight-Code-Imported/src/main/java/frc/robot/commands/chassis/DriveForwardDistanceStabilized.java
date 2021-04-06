// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;

import frc.robot.Constants;
import frc.robot.DeadReckoning;
import frc.robot.subsystems.Chassis;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveForwardDistanceStabilized extends PIDCommand {

  private final Chassis m_chassis;
  private final DeadReckoning m_deadReckoning;
  private double m_startDistanceMeters;
  private double m_distanceMeters;

  /** Creates a new DriveForwardDistanceStabilized. */
  public DriveForwardDistanceStabilized(Chassis chassis, double distanceMeters) {
    super(
      new PIDController(Constants.DRIVE_STABILIZE_P, Constants.DRIVE_STABILIZE_I, Constants.DRIVE_STABILIZE_D),
      // Close the loop on the turn rate
      chassis::getRate,
      // Setpoint is 0
      0,
      // Pipe the output to the turning controls
      output -> chassis.drive(Constants.AUTO_SPEED_FORWARD + output, Constants.AUTO_SPEED_FORWARD - output),
      // Require the robot drive
      chassis);

    m_chassis = chassis;
    m_deadReckoning = new DeadReckoning();
    m_distanceMeters = distanceMeters;
  }

  @Override
  public void initialize() {
    m_controller.reset();

    m_startDistanceMeters = m_chassis.getDistanceMeters();
  }

  @Override
  public void end(boolean interrupted) {
    m_useOutput.accept(0);

    double distanceMeters = m_chassis.getDistanceMeters() - m_startDistanceMeters;
    m_deadReckoning.printDriveForwardDistanceCmd(distanceMeters);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_chassis.getDistanceMeters() - m_startDistanceMeters > m_distanceMeters;

  }
}
