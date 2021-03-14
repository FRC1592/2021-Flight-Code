package frc.robot.commands.chassis;

import frc.robot.Constants;
import frc.robot.subsystems.Chassis;

public class RotateClockwise extends RotateConstantSpeed {
  public RotateClockwise(Chassis chassis) {
    super(chassis, Constants.AUTO_SPEED_ROTATE, -Constants.AUTO_SPEED_ROTATE, "Clockwise");
  }
}