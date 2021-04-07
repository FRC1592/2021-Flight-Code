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
import frc.robot.commands.autonomous.AutoBarrel;
import frc.robot.commands.autonomous.AutoBounce;
import frc.robot.commands.autonomous.AutoPaths;
import frc.robot.commands.autonomous.AutoSlalom;
import frc.robot.commands.chassis.DriveForwardConstantSpeed;
import frc.robot.commands.chassis.DriveForwardDistance;
import frc.robot.commands.chassis.DriveForwardDistanceStabilized;
import frc.robot.commands.chassis.DriveWithJoysticks;
import frc.robot.commands.chassis.RotateClockwise;
import frc.robot.commands.chassis.RotateCounterClockwise;
import frc.robot.commands.chassis.TurnLeft;
import frc.robot.commands.chassis.TurnRight;
// import frc.robot.commands.tomwheel.RotateColor;
// import frc.robot.commands.tomwheel.RotateCount;
import frc.robot.lib1592.hids.XBoxGamepad;
import frc.robot.lib1592.hids.XBoxButton.ButtonName;
import frc.robot.lib1592.utils.Discontinuities;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Shooter;
// import frc.robot.subsystems.TomWheel;

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
  
  // LIDAR
  private final LIDAR lidar = new LIDAR();

  // Subsystems
  private final Chassis m_chassis = new Chassis(lidar);
  private final Shooter m_shooter = new Shooter(m_joyDriver, this);
  //  final TomWheel m_tomwheel = new TomWheel();

  // Auto commands
  private final WaitCommand m_autoDoNothing = new WaitCommand(1.0);
  private final AutoBarrel m_autoBarrel = new AutoBarrel(m_chassis);
  private final AutoBounce m_autoBounce = new AutoBounce(m_chassis);
  private final AutoPaths m_autoPaths = new AutoPaths(m_chassis, m_shooter);
  private final AutoSlalom m_autoSlalom = new AutoSlalom(m_chassis);

  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser_autonomous = new SendableChooser<>();
  SendableChooser<Double> m_chooser_shooter = new SendableChooser<>();
  
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
    m_chooser_autonomous.setDefaultOption("Do nothing Auto", m_autoDoNothing);
    m_chooser_autonomous.addOption("Barrel Auto", m_autoBarrel);
    m_chooser_autonomous.addOption("Bounce Auto", m_autoBounce);
    m_chooser_autonomous.addOption("Paths Auto", m_autoPaths);
    m_chooser_autonomous.addOption("Slalom Auto", m_autoSlalom);

    // Add options to the auto chooser
    m_chooser_shooter.setDefaultOption("Default", Constants.SHOOTER_PERCENT_OUTPUT);
    m_chooser_shooter.addOption("Green", Constants.SHOOTER_PERCENT_OUTPUT_GREEN);
    m_chooser_shooter.addOption("Yellow", Constants.SHOOTER_PERCENT_OUTPUT_YELLOW);
    m_chooser_shooter.addOption("Blue", Constants.SHOOTER_PERCENT_OUTPUT_BLUE);
    m_chooser_shooter.addOption("Red", Constants.SHOOTER_PERCENT_OUTPUT_RED);
    m_chooser_shooter.addOption("Reintroduction", Constants.SHOOTER_PERCENT_OUTPUT_REINTRODUCTION);

    // Put the chooser on the dashboards
    Shuffleboard.getTab("Autonomous").add(m_chooser_autonomous);
    SmartDashboard.putData(m_chooser_autonomous);
    SmartDashboard.putData(m_chooser_shooter);
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
    // new JoystickButton(m_joyDriver, ButtonName.RIGHT_BUMPER.value)
    //     .whenPressed(new DriveForwardDistanceStabilized(m_chassis, 100.0));
    new JoystickButton(m_joyDriver, ButtonName.RIGHT_BUMPER.value)
        .whileHeld(new DriveForwardConstantSpeed(m_chassis, Constants.AUTO_SPEED_FORWARD));
    // new JoystickButton(m_joyDriver, ButtonName.X.value)
    //     .whenPressed(new TurnLeft(m_chassis));
    // new JoystickButton(m_joyDriver, ButtonName.B.value)
    //     .whenPressed(new TurnRight(m_chassis));
    new JoystickButton(m_joyDriver, ButtonName.X.value)
        .whileHeld(new RotateCounterClockwise(m_chassis));
    new JoystickButton(m_joyDriver, ButtonName.B.value)
        .whileHeld(new RotateClockwise(m_chassis));
    new JoystickButton(m_joyDriver, ButtonName.START.value)
        .whileHeld(new InstantCommand(m_chassis::zeroYaw, m_chassis));
        
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
    return m_chooser_autonomous.getSelected();
  }

  public Double getShooterOutput() {
    return m_chooser_shooter.getSelected();
  }

  public void stopGather() {
    m_shooter.stopGather();
  }
}