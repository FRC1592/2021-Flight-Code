/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.lib1592.control.PIDConstantsCTRE;

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
    public static final double MIN_TO_100ms = 1.0 / 600.0;  // Converts units per min to units per 100ms for the talons

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

    public static final boolean INVERT_GATHER = false;
    public static final boolean INVERT_KICKER = false;
    public static final boolean INVERT_SHOOTER = false;
    public static final boolean INVERT_SHOOTER_SENSOR = false;
    public static final boolean INVERT_LOADED = false;

    public static final int ENC_SHOOTER_PPR = 4096;     // Counts per rev

    public static final PIDConstantsCTRE PID_SHOOTER = new PIDConstantsCTRE(0.0, 0.0, 0.0, 1.0);
    public static final int ERROR_MAX_SHOOTER = 10;     // RPM

    public static final double SPEED_GATHER = 0.5;      // % Output
    public static final double SPEED_KICKER = 1.0;      // % Output
    public static final int SPEED_SHOOTER = 1600;       // RPM

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
}
