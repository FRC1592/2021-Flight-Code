/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.tomwheel;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TomWheel;
import frc.robot.subsystems.TomWheel.WheelColor;

public class RotateColor extends CommandBase {
  TomWheel m_tomwheel;
  WheelColor m_prevColor;
  WheelColor m_targetColor;
  boolean m_finished;

  public RotateColor(TomWheel tomwheel) {
    m_tomwheel = tomwheel;
  }

  @Override
  public void initialize() {
    m_prevColor = m_tomwheel.getColor();
    m_targetColor = m_tomwheel.getTargetColor();
    m_finished = false;
    m_tomwheel.startWheel();
  }

  @Override
  public void execute() {
    WheelColor m_curColor = m_tomwheel.getColor();
    if(m_curColor != m_prevColor) {
      if(m_curColor == m_targetColor)
        m_finished = true;
      SmartDashboard.putString("Memes: ", "New color state: m_prevColor = " + m_prevColor.value + ", m_curColor = " + m_curColor.value);
      m_prevColor = m_curColor;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_tomwheel.stopWheel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_finished;
  }
}
