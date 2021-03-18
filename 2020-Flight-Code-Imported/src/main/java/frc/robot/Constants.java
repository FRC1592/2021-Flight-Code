/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.util.Color;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //===================//
    //      Generic      //
    //===================//
    public static final int TALON_TIMEOUT = 10;             // 10ms
    public static final double MIN_TO_100_MS = 1.0 / 600.0;  // Converts units per min to units per 100ms for the talons
    public static final double MS_TO_SEC = 1.0 / 1000.0;
    
    //=====================//
    //      Joysticks      //
    //=====================//
    public static final double JOY_EXPO = 0.1;
    
    public static final int JOY_DRIVER = 0;
    public static final int JOY_MANIPULATOR = 1;
    
    //===================//
    //      Chassis      //
    //===================//
    public static final int ID_DRIVE_LMASTER = 1;       // DRV L MSTR 1
    public static final int ID_DRIVE_LSLAVE = 2;        // DRV L SLV 2
    public static final int ID_DRIVE_RMASTER = 3;       // DRV R MSTR 3
    public static final int ID_DRIVE_RSLAVE = 4;        // DRV R SLV 4
    
    public static final boolean INVERT_DRIVE = true;
    
    //===================//
    //      Shooter      //
    //===================//
    public static final int ID_GATHER = 1;      // GTHR 1
    public static final int ID_KICKER = 2;      // KICK 2
    public static final int ID_SHOOTER = 3;     // SHTR 3
    public static final int DIO_LOADED = 0;
    
    public static final boolean INVERT_GATHER = true;
    public static final boolean INVERT_KICKER = true;
    public static final boolean INVERT_SHOOTER = false;
    public static final boolean INVERT_SHOOTER_SENSOR = false;
    public static final boolean INVERT_LOADED = false;
    
    public static final int ENC_SHOOTER_PPR = 4096;     // Counts per rev
    
    /**
     * Which PID slot to pull gains from. Starting 2018, you can choose from
     * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
     * configuration.
	 */
    public static final int SLOT_IDX = 0;
    
	/**
     * Talon FX supports multiple (cascaded) PID loops. For
     * now we just want the primary one.
	 */
    public static final int PID_LOOP_IDX = 0;
    
	/**
     * Set to zero to skip waiting for confirmation, set to nonzero to wait and
     * report to DS if action fails.
	 */
    public static final int TALON_FX_TIMEOUT = 30;             // 30ms

    /**
	 * PID Gains may have to be adjusted based on the responsiveness of control loop.
     * kF: 1023 represents output value to Talon at 100%, 20660 represents Velocity units at 100% output
     * 
	 * 	                                    			  kP   	 kI    kD      kF          Iz    PeakOut */
    public final static Gains GAIN_VELOCITY  = new Gains( 0.1, 0.001, 5, 1023.0/20660.0,  300,  1.00);
    // https://docs.ctre-phoenix.com/en/latest/ch16_ClosedLoop.html#velocity-closed-loop-control-mode
    // https://docs.ctre-phoenix.com/en/latest/ch16_ClosedLoop.html?highlight=feed%20foward#calculating-velocity-feed-forward-gain-kf

    public static final double SHOOTER_PERCENT_OUTPUT = 0.5;
    public static final double SHOOTER_TARGET_RPM = 2000.0;

    /**
    * Convert RPM to units / 100ms.
    * 2048 Units/Rev / 600 100ms/min in either direction:
    * velocity setpoint is in units/100ms
    */
    public final static double RPM_TO_UNITS_PER_100MS = 2048.0 * MIN_TO_100_MS; // 1 RPM
    
    public static final int ERROR_MAX_SHOOTER = 10;     // RPM

    public static final double SPEED_GATHER = 0.5;      // % Output
    public static final double SPEED_KICKER = 1.0;      // % Output
    public static final double TARGET_VELOCITY_UNITS_PER_100_MS = Constants.SHOOTER_PERCENT_OUTPUT * Constants.SHOOTER_TARGET_RPM * Constants.RPM_TO_UNITS_PER_100MS; // 500 RPM

    //================-===//
    //      TomWheel      //
    //=================-==//
    public static final int ID_TOMLIFT = 4;     // TLIFT 4
    public static final int ID_TOMWHEEL = 5;    // TWHL 5
    public static final int DIO_TOM_LIMIT = 1;

    public static final boolean INVERT_TOM_LIFT = false;
    public static final boolean INVERT_TOM_WHEEL = false;
    public static final boolean INVERT_TOM_LIMIT = true;

    public static final double SPEED_TOM_LIFT = 1.0;       // % Output
    public static final double SPEED_TOM_WHEEL = 1.0;       // % Output

    public static final int DIST_COLOR_SENSOR_MAX  = 1000;

    //public static final Color COLOR_BLUE = ColorMatch.makeColor(0.143, 0.427, 0.429);
    //public static final Color COLOR_GREEN = ColorMatch.makeColor(0.197, 0.561, 0.240);
    //public static final Color COLOR_RED = ColorMatch.makeColor(0.561, 0.232, 0.114);
    //public static final Color COLOR_YELLOW = ColorMatch.makeColor(0.361, 0.524, 0.113);

    public static final Color COLOR_BLUE = ColorMatch.makeColor(0.150, 0.385, 0.463);
    public static final Color COLOR_GREEN = ColorMatch.makeColor(0.182, 0.578, 0.240);
    public static final Color COLOR_RED = ColorMatch.makeColor(0.630, 0.290, 0.084);
    public static final Color COLOR_YELLOW = ColorMatch.makeColor(0.246, 0.489, 0.262);

    //================-===//
    //      Autonomous     //
    //=================-==//
    public static final double AUTO_SPEED_FORWARD_BARREL = 0.6;
    public static final double AUTO_SPEED_FORWARD_SLALOM = 0.45;
    public static final double AUTO_SPEED_FORWARD = AUTO_SPEED_FORWARD_BARREL;
    public static final double AUTO_SPEED_ROTATE = 0.5;
}