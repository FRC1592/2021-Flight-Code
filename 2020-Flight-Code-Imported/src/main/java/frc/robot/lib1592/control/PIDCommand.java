/*----------------------------------------------------------------------------*/
/* Copyright (c) 2008-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.lib1592.control;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

/**
 * This class defines a {@link Command} which interacts heavily with a PID loop.
 *
 * <p> It provides some convenience methods to run an internal {@link PIDController} . It will also
 * start and stop said {@link PIDController} when the {@link PIDCommand} is first initialized and
 * ended/interrupted. </p>
 */
public abstract class PIDCommand extends CommandBase implements Sendable {
  /**
   * The internal {@link PIDController}.
   */
  private final PIDController m_controller;
  /**
   * An output which calls {@link PIDCommand#usePIDOutput(double)}.
   */
  private final PIDOutput m_output = this::usePIDOutput;
  /**
   * A source which calls {@link PIDCommand#returnPIDInput()}.
   */
  private final PIDSource m_source = new PIDSource() {
    public void setPIDSourceType(PIDSourceType pidSource) {
    }

    public PIDSourceType getPIDSourceType() {
      return PIDSourceType.kDisplacement;
    }

    public double pidGet() {
      return returnPIDInput();
    }
  };

  /**
   * Instantiates a {@link PIDCommand} that will use the given constants.
   *
   * @param name  the name
   * @param constants  the PID constants
   */
  public PIDCommand(String name, PIDConstants constants) {
    super(); // Removed name due to change from 2019 to 2020 command framework.
    m_controller = new PIDController(new BuilderPID(m_source, m_output, constants));
  }
  
  /**
   * Instantiates a {@link PIDCommand} that will use the given constants.
   *
   * @param constants  the PID constants
   */
  public PIDCommand(PIDConstants constants) {
    m_controller = new PIDController(new BuilderPID(m_source, m_output, constants));
  }

  /**
   * Returns the {@link PIDController} used by this {@link PIDCommand}. Use this if you would like
   * to fine tune the pid loop.
   *
   * @return the {@link PIDController} used by this {@link PIDCommand}
   */
  protected PIDController getPIDController() {
    return m_controller;
  }

  @Override
  public void initialize() {
    super.initialize();
    m_controller.setEnabled(true);
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    m_controller.setEnabled(false);
  }

  // Removed for 2020.
  // @Override
  // protected void interrupted() {
  //   end();
  // }

  /**
   * Adds the given value to the setpoint. If {@link PIDCommand#setInputRange(double, double)
   * setInputRange(...)} was used, then the bounds will still be honored by this method.
   *
   * @param deltaSetpoint the change in the setpoint
   */
  public void setSetpointRelative(double deltaSetpoint) {
    setSetpoint(getSetpoint() + deltaSetpoint);
  }

  /**
   * Sets the setpoint to the given value. If {@link PIDCommand#setInputRange(double, double)
   * setInputRange(...)} was called, then the given setpoint will be trimmed to fit within the
   * range.
   *
   * @param setpoint the new setpoint
   */
  protected void setSetpoint(double setpoint) {
    m_controller.setSetpoint(setpoint);
  }

  /**
   * Returns the setpoint.
   *
   * @return the setpoint
   */
  protected double getSetpoint() {
    return m_controller.getSetpoint();
  }

  /**
   * Returns the current position.
   *
   * @return the current position
   */
  protected double getPosition() {
    return returnPIDInput();
  }

  /**
   * Sets the maximum and minimum values expected from the input and setpoint.
   *
   * @param minimumInput the minimum value expected from the input and setpoint
   * @param maximumInput the maximum value expected from the input and setpoint
   */
  protected void setInputRange(double minimumInput, double maximumInput) {
    m_controller.setInputRange(minimumInput, maximumInput);
  }

  /**
   * Returns the input for the pid loop.
   *
   * <p>It returns the input for the pid loop, so if this command was based off of a gyro, then it
   * should return the angle of the gyro.
   *
   * <p>All subclasses of {@link PIDCommand} must override this method.
   *
   * <p>This method will be called in a different thread then the {@link Scheduler} thread.
   *
   * @return the value the pid loop should use as input
   */
  protected abstract double returnPIDInput();

  /**
   * Uses the value that the pid loop calculated. The calculated value is the "output" parameter.
   * This method is a good time to set motor values, maybe something along the lines of
   * <code>driveline.tankDrive(output, -output)</code>
   *
   * <p>All subclasses of {@link PIDCommand} must override this method.
   *
   * <p>This method will be called in a different thread then the {@link Scheduler} thread.
   *
   * @param output the value the pid loop calculated
   */
  protected abstract void usePIDOutput(double output);

  @Override
  public void initSendable(SendableBuilder builder) {
    m_controller.initSendable(builder);
    super.initSendable(builder);
    builder.setSmartDashboardType("PIDCommand");
  }
}
