package frc.robot.commands.chassis;

import frc.robot.subsystems.Chassis;

public class TurnLeft extends TurnToAngle {
  public TurnLeft(Chassis chassis) {
    super(chassis, -90.0);
  }
}