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
public class AutoPaths extends SequentialCommandGroup {
  private final Chassis m_chassis;

  /** Creates a new AutoPaths. */
  public AutoPaths(Chassis chassis, Shooter shooter) {
    m_chassis = chassis;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new StartGather(shooter)
      
    );
  }
}