// Copyright (c) Constants.AUTO_SPEED_ROTATE, -Constants.AUTO_SPEED_ROTATE and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Chassis;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoSlalom extends SequentialCommandGroup {
  private final Chassis m_chassis;

/** Creates a new AutoSlalom. */
  public AutoSlalom(Chassis chassis) {
    m_chassis = chassis;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveFowardConstantSpeed(m_chassis, Constants.AUTO_SPEED_FORWARD).withTimeout(958.0 / 1000.0),
      new WaitCommand(1.0),
      new RotateConstantSpeed(m_chassis, -Constants.AUTO_SPEED_ROTATE, Constants.AUTO_SPEED_ROTATE).withTimeout(959.0 / 1000.0),
      new WaitCommand(1.0),
      new DriveFowardConstantSpeed(m_chassis, Constants.AUTO_SPEED_FORWARD).withTimeout(1120.0 / 1000.0),
      new WaitCommand(1.0),
      new RotateConstantSpeed(m_chassis, Constants.AUTO_SPEED_ROTATE, -Constants.AUTO_SPEED_ROTATE).withTimeout(840.0 / 1000.0),
      new WaitCommand(1.0),
      new RotateConstantSpeed(m_chassis, Constants.AUTO_SPEED_ROTATE, -Constants.AUTO_SPEED_ROTATE).withTimeout(100.0 / 1000.0),
      new WaitCommand(1.0),
      new RotateConstantSpeed(m_chassis, Constants.AUTO_SPEED_ROTATE, -Constants.AUTO_SPEED_ROTATE).withTimeout(120.0 / 1000.0),
      new WaitCommand(1.0),
      new DriveFowardConstantSpeed(m_chassis, Constants.AUTO_SPEED_FORWARD).withTimeout(3340.0 / 1000.0),
      new WaitCommand(1.0),
      new RotateConstantSpeed(m_chassis, Constants.AUTO_SPEED_ROTATE, -Constants.AUTO_SPEED_ROTATE).withTimeout(780.0 / 1000.0),
      new WaitCommand(1.0),
      new DriveFowardConstantSpeed(m_chassis, Constants.AUTO_SPEED_FORWARD).withTimeout(1140.0 / 1000.0),
      new WaitCommand(1.0),
      new DriveFowardConstantSpeed(m_chassis, Constants.AUTO_SPEED_FORWARD).withTimeout(400.0 / 1000.0),
      new WaitCommand(1.0),
      new RotateConstantSpeed(m_chassis, -Constants.AUTO_SPEED_ROTATE, Constants.AUTO_SPEED_ROTATE).withTimeout(400.0 / 1000.0),
      new WaitCommand(1.0),
      new RotateConstantSpeed(m_chassis, -Constants.AUTO_SPEED_ROTATE, Constants.AUTO_SPEED_ROTATE).withTimeout(240.0 / 1000.0),
      new WaitCommand(1.0),
      new DriveFowardConstantSpeed(m_chassis, Constants.AUTO_SPEED_FORWARD).withTimeout(640.0 / 1000.0),
      new WaitCommand(1.0),
      new RotateConstantSpeed(m_chassis, -Constants.AUTO_SPEED_ROTATE, Constants.AUTO_SPEED_ROTATE).withTimeout(940.0 / 1000.0),
      new WaitCommand(1.0),
      new RotateConstantSpeed(m_chassis, -Constants.AUTO_SPEED_ROTATE, Constants.AUTO_SPEED_ROTATE).withTimeout(180.0 / 1000.0),
      new WaitCommand(1.0),
      new DriveFowardConstantSpeed(m_chassis, Constants.AUTO_SPEED_FORWARD).withTimeout(1080.0 / 1000.0),
      new WaitCommand(1.0),
      new RotateConstantSpeed(m_chassis, -Constants.AUTO_SPEED_ROTATE, Constants.AUTO_SPEED_ROTATE).withTimeout(900.0 / 1000.0),
      new WaitCommand(1.0),
      new DriveFowardConstantSpeed(m_chassis, Constants.AUTO_SPEED_FORWARD).withTimeout(500.0 / 1000.0),
      new WaitCommand(1.0),
      new RotateConstantSpeed(m_chassis, -Constants.AUTO_SPEED_ROTATE, Constants.AUTO_SPEED_ROTATE).withTimeout(1300.0 / 1000.0),
      new WaitCommand(1.0),
      new DriveFowardConstantSpeed(m_chassis, Constants.AUTO_SPEED_FORWARD).withTimeout(780.0 / 1000.0),
      new WaitCommand(1.0),
      new RotateConstantSpeed(m_chassis, -Constants.AUTO_SPEED_ROTATE, Constants.AUTO_SPEED_ROTATE).withTimeout(820.0 / 1000.0),
      new WaitCommand(1.0),
      new DriveFowardConstantSpeed(m_chassis, Constants.AUTO_SPEED_FORWARD).withTimeout(879.0 / 1000.0),
      new WaitCommand(1.0),
      new RotateConstantSpeed(m_chassis, Constants.AUTO_SPEED_ROTATE, -Constants.AUTO_SPEED_ROTATE).withTimeout(440.0 / 1000.0),
      new WaitCommand(1.0),
      new RotateConstantSpeed(m_chassis, Constants.AUTO_SPEED_ROTATE, -Constants.AUTO_SPEED_ROTATE).withTimeout(300.0 / 1000.0),
      new WaitCommand(1.0),
      new DriveFowardConstantSpeed(m_chassis, Constants.AUTO_SPEED_FORWARD).withTimeout(520.0 / 1000.0),
      new WaitCommand(1.0),
      new RotateConstantSpeed(m_chassis, Constants.AUTO_SPEED_ROTATE, -Constants.AUTO_SPEED_ROTATE).withTimeout(396.0 / 1000.0),
      new WaitCommand(1.0),
      new RotateConstantSpeed(m_chassis, Constants.AUTO_SPEED_ROTATE, -Constants.AUTO_SPEED_ROTATE).withTimeout(120.0 / 1000.0),
      new WaitCommand(1.0),
      new RotateConstantSpeed(m_chassis, Constants.AUTO_SPEED_ROTATE, -Constants.AUTO_SPEED_ROTATE).withTimeout(140.0 / 1000.0),
      new WaitCommand(1.0),
      new DriveFowardConstantSpeed(m_chassis, Constants.AUTO_SPEED_FORWARD).withTimeout(2960.0 / 1000.0),
      new WaitCommand(1.0),
      new DriveFowardConstantSpeed(m_chassis, Constants.AUTO_SPEED_FORWARD).withTimeout(280.0 / 1000.0),
      new WaitCommand(1.0),
      new RotateConstantSpeed(m_chassis, Constants.AUTO_SPEED_ROTATE, -Constants.AUTO_SPEED_ROTATE).withTimeout(600.0 / 1000.0),
      new WaitCommand(1.0),
      new DriveFowardConstantSpeed(m_chassis, Constants.AUTO_SPEED_FORWARD).withTimeout(640.0 / 1000.0),
      new WaitCommand(1.0),
      new RotateConstantSpeed(m_chassis, Constants.AUTO_SPEED_ROTATE, -Constants.AUTO_SPEED_ROTATE).withTimeout(100.0 / 1000.0),
      new WaitCommand(1.0),
      new DriveFowardConstantSpeed(m_chassis, Constants.AUTO_SPEED_FORWARD).withTimeout(820.0 / 1000.0),
      new WaitCommand(1.0),
      new RotateConstantSpeed(m_chassis, -Constants.AUTO_SPEED_ROTATE, Constants.AUTO_SPEED_ROTATE).withTimeout(460.0 / 1000.0),
      new WaitCommand(1.0),
      new RotateConstantSpeed(m_chassis, -Constants.AUTO_SPEED_ROTATE, Constants.AUTO_SPEED_ROTATE).withTimeout(140.0 / 1000.0),
      new WaitCommand(1.0),
      new DriveFowardConstantSpeed(m_chassis, Constants.AUTO_SPEED_FORWARD).withTimeout(1040.0 / 1000.0),
      new WaitCommand(1.0),
      new RotateConstantSpeed(m_chassis, -Constants.AUTO_SPEED_ROTATE, Constants.AUTO_SPEED_ROTATE).withTimeout(500.0 / 1000.0),
      new WaitCommand(1.0),
      new DriveFowardConstantSpeed(m_chassis, Constants.AUTO_SPEED_FORWARD).withTimeout(760.0 / 1000.0),
      new WaitCommand(1.0),
      new RotateConstantSpeed(m_chassis, -Constants.AUTO_SPEED_ROTATE, Constants.AUTO_SPEED_ROTATE).withTimeout(979.0 / 1000.0),
      new WaitCommand(1.0),
      new RotateConstantSpeed(m_chassis, -Constants.AUTO_SPEED_ROTATE, Constants.AUTO_SPEED_ROTATE).withTimeout(180.0 / 1000.0),
      new WaitCommand(1.0),
      new RotateConstantSpeed(m_chassis, -Constants.AUTO_SPEED_ROTATE, Constants.AUTO_SPEED_ROTATE).withTimeout(120.0 / 1000.0),
      new WaitCommand(1.0),
      new DriveFowardConstantSpeed(m_chassis, Constants.AUTO_SPEED_FORWARD).withTimeout(1120.0 / 1000.0),
      new WaitCommand(1.0),
      new RotateConstantSpeed(m_chassis, -Constants.AUTO_SPEED_ROTATE, Constants.AUTO_SPEED_ROTATE).withTimeout(1139.0 / 1000.0),
      new WaitCommand(1.0),
      new RotateConstantSpeed(m_chassis, Constants.AUTO_SPEED_ROTATE, -Constants.AUTO_SPEED_ROTATE).withTimeout(240.0 / 1000.0),
      new WaitCommand(1.0),
      new DriveFowardConstantSpeed(m_chassis, Constants.AUTO_SPEED_FORWARD).withTimeout(980.0 / 1000.0),
      new WaitCommand(1.0),
      new RotateConstantSpeed(m_chassis, -Constants.AUTO_SPEED_ROTATE, Constants.AUTO_SPEED_ROTATE).withTimeout(99.0 / 1000.0),
      new WaitCommand(1.0)
    );
  }
}