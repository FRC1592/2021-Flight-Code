// Copyright (c) Constants.AUTO_SPEED_ROTATE, -Constants.AUTO_SPEED_ROTATE and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.chassis.DriveForwardConstantSpeed;
import frc.robot.commands.shooter.StartGather;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoSlalom extends SequentialCommandGroup {
  private final Chassis m_chassis;
  private final Shooter m_shooter;

  /** Creates a new AutoSlalom. */
  public AutoSlalom(Chassis chassis, Shooter shooter) {
    m_chassis = chassis;
    m_shooter = shooter;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveForwardConstantSpeed(m_chassis, 0.6).withTimeout(1056.0 * Constants.MS_TO_SEC),
new DriveForwardConstantSpeed(m_chassis, 0.6).withTimeout(480.0 * Constants.MS_TO_SEC),
new RotateClockwise(m_chassis).withTimeout(139.0 * Constants.MS_TO_SEC),
new DriveForwardConstantSpeed(m_chassis, 0.6).withTimeout(400.0 * Constants.MS_TO_SEC),
new RotateClockwise(m_chassis).withTimeout(380.0 * Constants.MS_TO_SEC),
new RotateClockwise(m_chassis).withTimeout(280.0 * Constants.MS_TO_SEC),
new DriveForwardConstantSpeed(m_chassis, 0.6).withTimeout(499.0 * Constants.MS_TO_SEC),
new DriveForwardConstantSpeed(m_chassis, 0.6).withTimeout(459.0 * Constants.MS_TO_SEC),
new RotateCounterClockwise(m_chassis).withTimeout(279.0 * Constants.MS_TO_SEC),
new RotateClockwise(m_chassis).withTimeout(660.0 * Constants.MS_TO_SEC),
new RotateClockwise(m_chassis).withTimeout(459.0 * Constants.MS_TO_SEC),
new RotateClockwise(m_chassis).withTimeout(140.0 * Constants.MS_TO_SEC),
new DriveForwardConstantSpeed(m_chassis, 0.6).withTimeout(879.0 * Constants.MS_TO_SEC),
new RotateCounterClockwise(m_chassis).withTimeout(340.0 * Constants.MS_TO_SEC),
new RotateClockwise(m_chassis).withTimeout(920.0 * Constants.MS_TO_SEC),
new DriveForwardConstantSpeed(m_chassis, 0.6).withTimeout(200.0 * Constants.MS_TO_SEC),
new DriveForwardConstantSpeed(m_chassis, 0.6).withTimeout(1000.0 * Constants.MS_TO_SEC),
new DriveForwardConstantSpeed(m_chassis, 0.6).withTimeout(320.0 * Constants.MS_TO_SEC),
new RotateClockwise(m_chassis).withTimeout(860.0 * Constants.MS_TO_SEC),
new DriveForwardConstantSpeed(m_chassis, 0.6).withTimeout(800.0 * Constants.MS_TO_SEC),
new RotateClockwise(m_chassis).withTimeout(1280.0 * Constants.MS_TO_SEC),
new DriveForwardConstantSpeed(m_chassis, 0.6).withTimeout(540.0 * Constants.MS_TO_SEC),
new RotateClockwise(m_chassis).withTimeout(240.0 * Constants.MS_TO_SEC),
new RotateCounterClockwise(m_chassis).withTimeout(460.0 * Constants.MS_TO_SEC),
new DriveForwardConstantSpeed(m_chassis, 0.6).withTimeout(1460.0 * Constants.MS_TO_SEC),
new DriveForwardConstantSpeed(m_chassis, 0.6).withTimeout(300.0 * Constants.MS_TO_SEC),
new RotateCounterClockwise(m_chassis).withTimeout(180.0 * Constants.MS_TO_SEC),
new RotateClockwise(m_chassis).withTimeout(300.0 * Constants.MS_TO_SEC),
new RotateClockwise(m_chassis).withTimeout(100.0 * Constants.MS_TO_SEC),
new DriveForwardConstantSpeed(m_chassis, 0.6).withTimeout(240.0 * Constants.MS_TO_SEC) 
    );
  }
}