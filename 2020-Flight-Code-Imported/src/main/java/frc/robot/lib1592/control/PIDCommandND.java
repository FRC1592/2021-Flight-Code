/*----------------------------------------------------------------------------*/
/* Copyright (c) 2008-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.lib1592.control;

import java.util.ArrayList;
import java.util.List;

import frc.robot.lib1592.control.InterfacePID.PIDOutputND;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

/**
 * This class defines a {@link Command} which interacts heavily with a PID loop.
 *
 * <p> It provides some convenience methods to run an internal {@link PIDControllerND} . It will also
 * start and stop said {@link PIDControllerND} when the {@link PIDCommandND} is first initialized and
 * ended/interrupted. </p>
 */
public abstract class PIDCommandND extends CommandBase implements Sendable {
  /**
   * The internal {@link PIDControllerND}.
   */
  private final PIDControllerND m_controller;
  /**
   * An output which calls {@link PIDCommandND#usePIDOutput(double)}.
   */
  private final PIDOutputND m_output = this::usePIDOutput;

  /**
   * A source which calls {@link PIDCommandND#returnPIDInput(int)}.
   */
  private final class InnerSource implements PIDSource {
	  private final int _index;
	  InnerSource(int index) {
		  _index = index;
	  }

	  @Override public void setPIDSourceType(PIDSourceType pidSource) {}

	  @Override public PIDSourceType getPIDSourceType() {
		  return PIDSourceType.kDisplacement;
	  }

	  @Override public double pidGet() {
		  return returnPIDInput(_index);
	  }
  };

  /**
   * Instantiates a {@link PIDCommandND} that will use the given constants.
   *
   * @param name  the name
   * @param constants  the PID constants
   */
  public PIDCommandND(String name, long period, PIDConstants... constants) {
    super(); // Removed name due to change from 2019 to 2020 command framework.
    m_controller = buildController(period, constants);
  }
  
  /**
   * Instantiates a {@link PIDCommandND} that will use the given constants.
   *
   * @param constants  the PID constants
   */
  public PIDCommandND(long period, PIDConstants constants) {
	  m_controller = buildController(period, constants);
  }
  
  /** Internal Builder */
  private final PIDControllerND buildController(long period, PIDConstants... constants) {
	  List<BuilderPID> builders = new ArrayList<BuilderPID>(constants.length);
	  int index = 0;
	  for (PIDConstants pc : constants) {
		  builders.add(new BuilderPID(new InnerSource(index++), (v) -> {}, pc));
	  }
	  PIDControllerND out = new PIDControllerND(period, builders);
	  out.setOutput(m_output);
	  return out;
  }

  /**
   * Returns the {@link PIDController} used by this {@link PIDCommandND}. Use this if you would like
   * to fine tune the pid loop.
   *
   * @return the {@link PIDController} used by this {@link PIDCommandND}
   */
  protected PIDControllerND getPIDController() {
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
   * Adds the given value to the setpoint. If {@link PIDCommandND#setInputRange(double, double)
   * setInputRange(...)} was used, then the bounds will still be honored by this method.
   *
   * @param deltaSetpoint the change in the setpoint
   */
  public void setSetpointRelative(int index, double deltaSetpoint) {
	  double[] set = getSetpoint();
    setSetpoint(index, set[index] + deltaSetpoint);
  }

  /**
   * Sets the setpoint to the given value.
   *
   * @param setpoint the new setpoint
   */
  protected void setSetpoint(int index, double setpoint) {
    m_controller.setSetpoint(index, setpoint);
  }
  
  /**
   * Sets the setpoints to the given value.
   *
   * @param setpoints the new setpoints
   */
  protected void setSetpoints(double... setpoints) {
    m_controller.setSetpoints(setpoints);
  }

  /**
   * Returns the setpoint.
   *
   * @return the setpoint
   */
  protected double[] getSetpoint() {
    return m_controller.getSetpoint();
  }

  /**
   * Returns the current position.
   *
   * @return the current position
   */
  protected double[] getPosition() {
	  double[] out = new double[m_controller.getDimensions()];
	  for (int index = 0; index < out.length; index++) {
		  out[index++] = returnPIDInput(index);
	  }
	  return out;
  }

  /**
   * Returns the input for the pid loop.
   *
   * <p>It returns the input for the pid loop, so if this command was based off of a gyro, then it
   * should return the angle of the gyro.
   *
   * <p>All subclasses of {@link PIDCommandND} must override this method.
   *
   * <p>This method will be called in a different thread then the {@link CommandScheduler} thread.
   *
   * @param index  the pid dimension
   * @return the value the pid loop should use as input
   */
  protected abstract double returnPIDInput(int index);

  /**
   * Uses the value that the pid loop calculated. The calculated value is the "output" parameter.
   * This method is a good time to set motor values, maybe something along the lines of
   * <code>driveline.tankDrive(output, -output)</code>
   *
   * <p>All subclasses of {@link PIDCommandND} must override this method.
   *
   * <p>This method will be called in a different thread then the {@link CommandScheduler} thread.
   *
   * @param output the value the pid loop calculated
   */
  protected abstract void usePIDOutput(double... output);

  @Override
  public void initSendable(SendableBuilder builder) {
    m_controller.initSendable(builder);
    super.initSendable(builder);
    builder.setSmartDashboardType("PIDCommand");
  }
}
