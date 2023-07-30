// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.lib.util.COTSFalconSwerveConstants;
import frc.robot.lib.util.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static final double STICK_DEADBAND = 0.25;

  public static final class CAN_IDS {
    /* Motor + sensor IDs */
    public static final int PIDGEON = 5;
  }

  public static final class DriverControls {
    /*
     * Axes to control drive base in teleop mode
     */
    public static final int TRANSLATION_VAL = XboxController.Axis.kLeftY.value;
    public static final int STRAFE_VAL = XboxController.Axis.kLeftX.value;
    public static final int ROTATION_VAL = XboxController.Axis.kRightX.value;

    /*
     * in teleop state:
     *  Zeros the gyro of the robot and changes its field alignment
     */
    public static final int ZERO_GYRO = XboxController.Button.kX.value;
  }

  public static final class SWERVE {
    public static final boolean INVERT_GYRO = false; // Always ensure Gyro is CCW+ CW-

    public static final COTSFalconSwerveConstants CHOSEN_MODULE = 
        COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

    /* Drivetrain Constants */
    public static final double TRACK_WIDTH = Units.inchesToMeters(22.0);
    public static final double WHEEL_BASE = Units.inchesToMeters(22.0); 
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(3.9);
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

    /* Swerve Kinematics 
     * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
     public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
        new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
        new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
        new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

    /* Module Gear Ratios */
    public static final double DRIVE_GEAR_RATIO = CHOSEN_MODULE.driveGearRatio;
    public static final double ANGLE_GEAR_RATIO = CHOSEN_MODULE.angleGearRatio;

    /* Motor Inverts */
    public static final boolean ANGLE_MOTOR_INVERT = CHOSEN_MODULE.angleMotorInvert;
    public static final boolean DRIVE_MOTOR_INVERT = CHOSEN_MODULE.driveMotorInvert;

    /* Angle Encoder Invert */
    public static final boolean CANCODER_INVERT = CHOSEN_MODULE.canCoderInvert;

    /* Swerve Current Limiting */
    public static final int ANGLE_CONTINUOUS_CURRENT_LIMIT = 25;
    public static final int ANGLE_PEAK_CURRENT_LIMIT = 40;
    public static final double ANGLE_PEAK_CURRENT_DURATION = 0.1;
    public static final boolean ANGLE_ENABLE_CURRENT_LIMIT = true;

    public static final int DRIVE_CONTINUOUS_CURRENT_LIMIT = 35;
    public static final int DRIVE_PEAK_CURRENT_LIMIT = 60;
    public static final double DRIVE_PEAK_CURRENT_DURATION = 0.1;
    public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

    /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
     * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
    public static final double OPEN_LOOP_RAMP = 0.25;
    public static final double CLOSED_LOOP_RAMP = 0.0;

    /* Angle Motor PID Values */
    public static final double ANGLE_KP = CHOSEN_MODULE.angleKP;
    public static final double ANGLE_KI = CHOSEN_MODULE.angleKI;
    public static final double ANGLE_KD = CHOSEN_MODULE.angleKD;
    public static final double ANGLE_KF = CHOSEN_MODULE.angleKF;

    /* Drive Motor PID Values */
    public static final double DRIVE_KP = 0.0;
    public static final double DRIVE_KI = 0.0;
    public static final double DRIVE_KD = 0.0;
    public static final double DRIVE_KF = 0.0;

    /* Drive Motor Characterization Values 
     * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
    public static final double DRIVE_KS = (0.0 / 12); 
    public static final double DRIVE_KV = (0.0 / 12);
    public static final double DRIVE_KA = (0.0 / 12);

    /* Swerve Profiling Values */
    /** Meters per Second */
    public static final double MAX_SPEED = 3.25; 
    /** Radians per Second */
    public static final double MAX_ANGULAR_VELOCITY = 4.0;

    /* Neutral Modes */
    public static final NeutralMode ANGLE_NEUTRAL_MODE = NeutralMode.Coast;
    public static final NeutralMode DRIVE_NEUTRAL_MODE = NeutralMode.Brake;
  
     /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 { 
      public static final int driveMotorID = 11;
      public static final int angleMotorID = 12;
      public static final int canCoderID = 10;
    // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
      public static final SwerveModuleConstants constants = 
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID);
  }

  /* Front Right Module - Module 1 */
  public static final class Mod1 {
    public static final int driveMotorID = 21;
    public static final int angleMotorID = 22;
    public static final int canCoderID = 20;
    // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
    public static final SwerveModuleConstants constants = 
        new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID);
}

/* Back Left Module - Module 2 */
public static final class Mod2 {
    public static final int driveMotorID = 31;
    public static final int angleMotorID = 32;
    public static final int canCoderID = 30;
    // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
    public static final SwerveModuleConstants constants = 
        new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID);
}

/* Back Right Module - Module 3 */
public static final class Mod3 { 
    public static final int driveMotorID = 41;
    public static final int angleMotorID = 42;
    public static final int canCoderID = 40;
    // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
    public static final SwerveModuleConstants constants = 
        new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID);
  }
  }

public static final Mode currentMode = Mode.REAL;

public static enum Mode {
/** Running on a real robot. */
REAL,

/** Running a physics simulator. */
SIM,

/** Replaying from a log file. */
REPLAY
}
}

