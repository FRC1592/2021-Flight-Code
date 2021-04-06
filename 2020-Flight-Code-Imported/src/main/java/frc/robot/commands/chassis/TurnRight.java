package frc.robot.commands.chassis;

import frc.robot.subsystems.Chassis;

public class TurnRight extends TurnToAngle {
  public TurnRight(Chassis chassis) {
    super(chassis, 90.0);
  }
}