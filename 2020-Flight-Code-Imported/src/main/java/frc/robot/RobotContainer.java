/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.chassis.AutoSlalom;
import frc.robot.commands.chassis.DriveForwardConstantSpeed;
import frc.robot.commands.chassis.DriveWithJoysticks;
import frc.robot.commands.chassis.RotateClockwise;
import frc.robot.commands.chassis.RotateCounterClockwise;
import frc.robot.commands.shooter.StartGather;
// import frc.robot.commands.tomwheel.RotateColor;
// import frc.robot.commands.tomwheel.RotateCount;
import frc.robot.lib1592.hids.XBoxGamepad;
import frc.robot.lib1592.hids.XBoxButton.ButtonName;
import frc.robot.lib1592.utils.Discontinuities;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TomWheel;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Controllers
  public final static XBoxGamepad m_joyDriver = new XBoxGamepad(Constants.JOY_DRIVER);
  public final static XBoxGamepad m_joyManipulator = new XBoxGamepad(Constants.JOY_MANIPULATOR);

  // Subsystems
  private final Chassis m_chassis = new Chassis();
  private final Shooter m_shooter = new Shooter(m_joyDriver);
  //  final TomWheel m_tomwheel = new TomWheel();

  // Auto commands
  private final WaitCommand m_autoDoNothing = new WaitCommand(1.0);
  private final AutoSlalom m_autoSlalom = new AutoSlalom(m_chassis, m_shooter);

  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Default commands
    m_chassis.setDefaultCommand(
      new DriveWithJoysticks(
        m_chassis,
        () -> Discontinuities.joyExpo(m_joyDriver.getY(Hand.kLeft), Constants.JOY_EXPO),
        () -> Discontinuities.joyExpo(m_joyDriver.getY(Hand.kRight), Constants.JOY_EXPO)));

    // Add options to the auto chooser
    m_chooser.setDefaultOption("Do nothing Auto", m_autoDoNothing);
    m_chooser.addOption("Slalom Auto", m_autoSlalom);

    // Put the chooser on the dashboards
    Shuffleboard.getTab("Autonomous").add(m_chooser);
    SmartDashboard.putData(m_chooser);
  }

  // Button -> command mappingss
  private void configureButtonBindings() {
    // Driver
    new JoystickButton(m_joyDriver, ButtonName.Y.value).whenPressed(new InstantCommand(m_shooter::startShooter, m_shooter));
    new JoystickButton(m_joyDriver, ButtonName.A.value).whenPressed(new InstantCommand(m_shooter::stopShooter, m_shooter));
    new JoystickButton(m_joyDriver, ButtonName.RIGHT_TRIGGER.value)
        .whenPressed(new InstantCommand(m_shooter::startKicker, m_shooter))
        .whenReleased(new InstantCommand(m_shooter::stopKicker, m_shooter));
    new JoystickButton(m_joyDriver, ButtonName.LEFT_TRIGGER.value)
        .whenPressed(new InstantCommand(m_shooter::startGather, m_shooter)) //.withInterrupt(m_shooter::isLoaded).andThen(m_shooter::stopGather, m_shooter)
        .whenReleased(new InstantCommand(m_shooter::stopGather, m_shooter));
    new JoystickButton(m_joyDriver, ButtonName.LEFT_BUMPER.value)
        .whenPressed(new InstantCommand(m_shooter::reverseGather, m_shooter))
        .whenReleased(new InstantCommand(m_shooter::stopGather, m_shooter));
    new JoystickButton(m_joyDriver, ButtonName.RIGHT_BUMPER.value)
        .whileHeld(new DriveForwardConstantSpeed(m_chassis, Constants.AUTO_SPEED_FORWARD));
    new JoystickButton(m_joyDriver, ButtonName.X.value)
        .whileHeld(new RotateCounterClockwise(m_chassis));
    new JoystickButton(m_joyDriver, ButtonName.B.value)
        .whileHeld(new RotateClockwise(m_chassis));
    new JoystickButton(m_joyDriver, ButtonName.START.value)
        .whenPressed(new StartGather(m_shooter));
        
    // Manipulator
    // new JoystickButton(m_joyManipulator, ButtonName.A.value)
    //     .whileActiveOnce(
    //       new StartEndCommand(m_tomwheel::lower, m_tomwheel::stopLift, m_tomwheel).withTimeout(1).andThen(
    //       new StartEndCommand(m_tomwheel::lower, m_tomwheel::stopLift, m_tomwheel).withInterrupt(m_tomwheel::limitHit)));

    // new JoystickButton(m_joyManipulator, ButtonName.Y.value)
    //     .whileActiveOnce(
    //       new StartEndCommand(m_tomwheel::raise, m_tomwheel::stopLift, m_tomwheel).withTimeout(1).andThen(
    //       new StartEndCommand(m_tomwheel::raise, m_tomwheel::stopLift, m_tomwheel).withInterrupt(m_tomwheel::limitHit),
    //       new StartEndCommand(m_tomwheel::raise, m_tomwheel::stopLift, m_tomwheel).withInterrupt(() -> !m_tomwheel.limitHit())
    //     ));

    // new JoystickButton(m_joyManipulator, ButtonName.X.value).whenPressed(new RotateCount(m_tomwheel));
    // new JoystickButton(m_joyManipulator, ButtonName.B.value).whenPressed(new RotateColor(m_tomwheel));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}