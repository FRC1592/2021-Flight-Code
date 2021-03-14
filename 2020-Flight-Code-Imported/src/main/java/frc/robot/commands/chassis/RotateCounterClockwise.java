package frc.robot.commands.chassis;

import frc.robot.Constants;
import frc.robot.subsystems.Chassis;

public class RotateCounterClockwise extends RotateConstantSpeed {
  public RotateCounterClockwise(Chassis chassis) {
    super(chassis, -Constants.AUTO_SPEED_ROTATE, Constants.AUTO_SPEED_ROTATE, "CounterClockwise");
  }
}