/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib1592.drivers.TalonFactory;
import frc.robot.lib1592.drivers.TalonSRX;
import frc.robot.lib1592.utils.Discontinuities;

public class TomWheel extends SubsystemBase {
  private final TalonSRX m_lift = TalonFactory.create(Constants.ID_TOMLIFT);
  private final TalonSRX m_wheel = TalonFactory.create(Constants.ID_TOMWHEEL);
  private final DigitalInput m_limit = new DigitalInput(Constants.DIO_TOM_LIMIT);
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(Port.kOnboard);
  private final ColorMatch m_colorMatcher = new ColorMatch();

  public TomWheel() {
    m_lift.setInverted(Constants.INVERT_TOM_LIFT);
    m_wheel.setInverted(Constants.INVERT_TOM_WHEEL);

    m_colorMatcher.addColorMatch(Constants.COLOR_RED);
    m_colorMatcher.addColorMatch(Constants.COLOR_GREEN);
    m_colorMatcher.addColorMatch(Constants.COLOR_BLUE);
    m_colorMatcher.addColorMatch(Constants.COLOR_YELLOW);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Proximity Raw", m_colorSensor.getProximity());
    SmartDashboard.putNumber("Proximity Scled", getDistance());
    SmartDashboard.putBoolean("Limit", limitHit());
    SmartDashboard.putString("Color", getColor().value);
    Color c = m_colorSensor.getColor();
    SmartDashboard.putString("Color raw", "(" + c.red + ", " + c.green + ", " + c.blue + ")");
    SmartDashboard.putBoolean("Position correct", positionCorrect());
  }

  public void startWheel() {
    m_wheel.set(ControlMode.PercentOutput, Constants.SPEED_TOM_WHEEL);
  }

  public void stopWheel() {
    m_wheel.stopMotor();
  }

  public void raise() {
    m_lift.set(ControlMode.PercentOutput, Constants.SPEED_TOM_LIFT);
  }

  public void lower() {
    m_lift.set(ControlMode.PercentOutput, -Constants.SPEED_TOM_LIFT);
  }

  public void stopLift() {
    m_lift.stopMotor();
  }

  public boolean limitHit() {
    return m_limit.get() ^ Constants.INVERT_TOM_LIMIT;
  }

  public WheelColor getColor() {
    if(!positionCorrect()) return WheelColor.UNKNOWN;

    ColorMatchResult match = m_colorMatcher.matchClosestColor(m_colorSensor.getColor());

    if (match.color == Constants.COLOR_BLUE) {
      return WheelColor.BLUE;
    } else if (match.color == Constants.COLOR_RED) {
      return WheelColor.RED;
    } else if (match.color == Constants.COLOR_GREEN) {
      return WheelColor.GREEN;
    } else if (match.color == Constants.COLOR_YELLOW) {
      return WheelColor.YELLOW;
    } else {
      return WheelColor.UNKNOWN;
    }
  }

  public double getDistance() {
    return 2047 - m_colorSensor.getProximity();
  }

  public boolean positionCorrect() {
    return Discontinuities.isBetween(getDistance(), -1, Constants.DIST_COLOR_SENSOR_MAX);
  }

  public WheelColor getTargetColor() {
    String gameData;
    gameData = DriverStation.getInstance().getGameSpecificMessage();
    if(gameData.length() > 0)
    {
      switch (gameData.charAt(0))
      {
        case 'B' :
          return WheelColor.BLUE;
        case 'G' :
          return WheelColor.GREEN;
        case 'R' :
          return WheelColor.RED;
        case 'Y' :
          return WheelColor.YELLOW;
        default :
          return WheelColor.UNKNOWN;
      }
    } else {
      return WheelColor.UNKNOWN;
    }
  }

  public enum WheelColor {
    RED("Red"),
    GREEN("Green"),
    BLUE("Blue"),
    YELLOW("Yellow"),
    UNKNOWN("Unknown");

		public final String value;

		private WheelColor(String color) {
			this.value = color;
		}
  };
}
