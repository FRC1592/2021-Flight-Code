// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.DeadReckoning;
import frc.robot.subsystems.Chassis;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnToAngle extends PIDCommand {
    
    private static final double kTurnP = 1.0;
    private static final double kTurnI = 0.0;
    private static final double kTurnD = 0.0;
    
    private static final double kTurnToleranceDeg = 5.0;
    private static final double kTurnRateToleranceDegPerS = 10.0;
    
    private final double m_angleDegrees;
    private final DeadReckoning m_deadReckoning;

	/** Creates a new TurnToAngle. */
    public TurnToAngle(Chassis chassis, double angleDegrees) {
        super(
            // The controller that the command will use
            new PIDController(kTurnP, kTurnI, kTurnD),
            // This should return the measurement
            // Close loop on heading
            () -> chassis.getAngle(),
            // This should return the setpoint (can also be a constant)
            // Set reference to target
            () -> {
                double currentAngleDegrees = chassis.getAngle();
                double setpointAngleDegrees = currentAngleDegrees + angleDegrees;
                return setpointAngleDegrees;
            },
            // This uses the output
            // Pipe output to turn robot
            output -> {
            // Use the output here
            chassis.drive(output, -output);
            }, chassis);

        // Configure additional PID options by calling `getController` here.

        // Set the controller to be continuous (because it is an angle controller)
        getController().enableContinuousInput(-180, 180);
        // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
        // setpoint before it is considered as having reached the reference
        getController()
            .setTolerance(kTurnToleranceDeg, kTurnRateToleranceDegPerS);

        m_angleDegrees = angleDegrees;
        m_deadReckoning = new DeadReckoning();
    }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    if (getController().atSetpoint()) {
        if (m_angleDegrees > 0.0) {
            m_deadReckoning.printTurnRightCmd();
        } else {
            m_deadReckoning.printTurnLeftCmd();
        }

        return true;
    }

    return false;
  }
}
