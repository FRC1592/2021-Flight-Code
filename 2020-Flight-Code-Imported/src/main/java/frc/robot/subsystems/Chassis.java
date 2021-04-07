/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LIDAR;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Chassis extends SubsystemBase {
  private final CANSparkMax m_leftMaster = new CANSparkMax(Constants.ID_DRIVE_LMASTER, MotorType.kBrushless);
  private final CANSparkMax m_leftSlave = new CANSparkMax(Constants.ID_DRIVE_LSLAVE, MotorType.kBrushless);

  private final CANSparkMax m_rightMaster = new CANSparkMax(Constants.ID_DRIVE_RMASTER, MotorType.kBrushless);
  private final CANSparkMax m_rightSlave = new CANSparkMax(Constants.ID_DRIVE_RSLAVE, MotorType.kBrushless);

  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMaster, m_rightMaster);

  /**
   * A CANEncoder object is constructed using the GetEncoder() method on an 
   * existing CANSparkMax object. The assumed encoder type is the hall effect,
   * or a sensor type and counts per revolution can be passed in to specify
   * a different kind of sensor. Here, it's a quadrature encoder with 4096 CPR.
   */
  private CANEncoder m_encoder;

  private final AHRS ahrs;

  private final LIDAR m_lidar;

  public Chassis(LIDAR lidar) {
    m_leftMaster.setInverted(Constants.INVERT_DRIVE);
    m_rightMaster.setInverted(Constants.INVERT_DRIVE);

    m_leftSlave.follow(m_leftMaster);
    m_rightSlave.follow(m_rightMaster);

    ahrs = new AHRS(SPI.Port.kMXP);

    m_encoder = m_leftMaster.getEncoder();

    m_lidar = lidar;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    /* These functions are compatible w/the WPI Gyro Class, providing a simple  */
    /* path for upgrading from the Kit-of-Parts gyro to the navx-MXP            */
    
    SmartDashboard.putNumber(   "IMU_TotalYaw",         ahrs.getAngle());
    SmartDashboard.putNumber(   "IMU_YawRateDPS",       ahrs.getRate());

    SmartDashboard.putNumber("Left drive motor encoder", m_encoder.getPosition());

    double distanceMeters = m_lidar.getDistanceMeters();

    SmartDashboard.putNumber("LIDAR", distanceMeters);

    double pathABlueDistanceMeters = 4.572;
    double pathARedDistanceMeters;
    double pathBBlueDistanceMeters = 5.334;
    double pathBRedDistanceMeters = 1.524;

    if (distanceMeters < pathBRedDistanceMeters * 1.1) {
      SmartDashboard.putString("Path", "B Red");
    } else if (distanceMeters < pathABlueDistanceMeters * 1.1) {
      SmartDashboard.putString("Path", "A Blue");
    } else if (distanceMeters < pathBBlueDistanceMeters * 1.1) {
      SmartDashboard.putString("Path", "B Blue");
    } else {
      SmartDashboard.putString("Path", "A Red");
    }
  }

  /**
   * Calls DifferentialDrive's tankDrive() method.
   */
  public void drive(double left, double right){
    m_drive.tankDrive(left, right);
  }

  public double getAngle() {
    return ahrs.getAngle();
  }

  public double getRate() {
    return ahrs.getRate();
  }

  public double getDistanceMeters() {
    return m_encoder.getPosition() * Constants.DRIVE_ENCODER_UNITS_TO_METERS;
  }

  public void zeroYaw() {
    ahrs.zeroYaw();
  }
}
