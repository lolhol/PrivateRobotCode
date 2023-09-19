// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class DriveConstants {

    // 4765: Used our addresses
    public static final int kFrontLeftDriveMotorPort = 10;
    public static final int kRearLeftDriveMotorPort = 21;
    public static final int kFrontRightDriveMotorPort = 12;
    public static final int kRearRightDriveMotorPort = 13;

    public static final int kFrontLeftTurningMotorPort = 22;
    public static final int kRearLeftTurningMotorPort = 25;
    public static final int kFrontRightTurningMotorPort = 20;
    public static final int kRearRightTurningMotorPort = 23;

    public static final int kFrontLeftTurningEncoderPort = 1;
    public static final int kRearLeftTurningEncoderPort = 4;
    public static final int kFrontRightTurningEncoderPort = 2;
    public static final int kRearRightTurningEncoderPort = 3;

    // 4765: determined by trial and error - probably right?

    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kRearLeftTurningEncoderReversed = false;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kRearRightTurningEncoderReversed = false;

    // 4765: determined by trial and error - probably right?

    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kRearLeftDriveEncoderReversed = false;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kRearRightDriveEncoderReversed = false;

    // 4765: figured out by using Phoenix Tuner app

    // Final(?) calibration with arm in final(?) orientation
    public static final double kFrontLeftTurningMagnetOffset = 21.7;
    public static final double kRearLeftTurningMagnetOffset = -88.0;
    public static final double kFrontRightTurningMagnetOffset = 209.25;
    public static final double kRearRightTurningMagnetOffset = -145.45;

    // less old
    // public static final double kFrontLeftTurningMagnetOffset = -57.75;
    // public static final double kRearLeftTurningMagnetOffset = -61.98;
    // public static final double kFrontRightTurningMagnetOffset = 1.84;
    // public static final double kRearRightTurningMagnetOffset = 111.11;

    // old
    // public static final double kFrontLeftTurningMagnetOffset = -62.5;
    // public static final double kRearLeftTurningMagnetOffset = 110.75;
    // public static final double kFrontRightTurningMagnetOffset = -56.2;
    // public static final double kRearRightTurningMagnetOffset = 1.75;

    // 4765: Measured by Justin!

    public static final double kTrackWidth = 0.47625;
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = 0.47625;
    // Distance between front and back wheels on robot

    // 4765 NOTE: WPILIB coordinate system is reportedly: x forward, y left, z up,
    // CCW positive angles

    // 4765: If behavior is backwards/upside-down feeling, explore messing with
    // this. Seems right currently.
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
    );

    // 4765 TODO: Need to fix and configure gyro or navex for field (driver) centric
    // driving
    public static final boolean kGyroReversed = false;

    // 4765: commented out because I think it has to do with odometry

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or
    // theoretically
    // for *your* robot's drive.
    // The SysId tool provides a convenient method for obtaining these values for
    // your robot.
    // public static final double ksVolts = 1;
    // public static final double kvVoltSecondsPerMeter = 0.8;
    // public static final double kaVoltSecondsSquaredPerMeter = 0.15;

    // 4765: Not sure what this should be configured to be, but made it slow for
    // safe testing during development
    // 4765 TODO: deteremine what this should be set to
    public static final double kMaxSpeedMetersPerSecond = 4;

    // 4765: Introduced some deadband constants (remember that X is forward!)
    public static final double kXSpeedDeadband = 0.05;
    public static final double kYSpeedDeadband = 0.05;
    public static final double kRotDeadband = 0.05;
  }

  public static final class ModuleConstants {

    // 4765 TODO: determine what these should actuall be
    public static final double kMaxModuleAngularSpeedRadiansPerSecond =
      2 * Math.PI * 24.1;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared =
      2 * Math.PI * 24.1;

    // 4765 TODO: determine what these should actually be
    // 4765 TODO: determine what effect this has (or is it just odometry)

    public static final int kEncoderCPR = 4096;
    public static final double kWheelDiameterMeters = 0.15;
    public static final double kDriveEncoderDistancePerPulse =
      // Assumes the encoders are directly mounted on the wheel shafts
      (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

    // 4765 TODO: determine what effect this has (or is it just odometry)
    public static final double kTurningEncoderDistancePerPulse =
      // Assumes the encoders are on a 1:1 reduction with the module shaft.
      (2 * Math.PI) / (double) kEncoderCPR;

    // 4765 TODO: Test and tune this. Important!!!

    // Drive PID P value
    public static final double kPModuleDriveController = 0.8;

    // 4765 TODO: Test and tune this. Important!!!

    // Turning PID P value
    public static final double kPModuleTurningController = 0.05;
  }

  public static final class ArmConstants {

    public static final double kPitchLevel = 0;
    //reach full extention 10.5
    //reach full retraction 0.15

    //pitch upper limit -0.1
    //pitch lower limit

    // first two, centered on grid tag
    // second two, centered on grid tag distanced to rotate
    // third two, rotated to the right (for scoring on left)

    // grid
    // yaw -125.1
    // rotated yaw -103.73
    // driven rotated yaw -103.5
    // diference 22

    // shelf
    // driven rotated yaw 96.12
    // centered 64.77
    // rotated yaw 97.74
    // difference 30

    // fourth four, centered not distanced
    // fifth four, centered distanced
    // sixth four, rotated
    // seventh four, rotated and driven
    // eighth four, centered shelf
    // ninth four, rotated and driven shelf

    //top shelf mod pitch 0.162
    //top shelf mod reach

    //top cone mod pitch 1.65
    //top cone mod reach 9.84

    //mid code mod pitch .153
    //mid cone mod reach 3.47

    //bottom cone and cube mod pitch -0.094
    //bottom cone and cube mod reach 0.15

    //mid cube mod pitch 0.0858
    //mid cube mod reach 2.426

    //top cube mod pitch 0.124
    //top cuve mod reach 8.739

    //bottom cube mod pitch
    //bottom cube mod reach

    public static final double kPitchConeTopScoreMod = 1;
    public static final double kPitchConeMidScoreMod = 1;
    public static final double kPitchConeBotScoreMod = -1;

    public static final double kPitchCubeTopScoreMod = 1;
    public static final double kPitchCubeMidScoreMod = 1;
    public static final double kPitchCubeBotScoreMod = -1;

    public static final double kReachConeTopScoreMod = 1;
    public static final double kReachConeMidScoreMod = 1;
    public static final double kReachConeBotScoreMod = 1;

    public static final double kReachCubeTopScoreMod = 1;
    public static final double kReachCubeMidScoreMod = 1;
    public static final double kReachCubeBotScoreMod = 1;

    public static final double kPitchTopShelfMod = 1;
    public static final double kReachTopShelfMod = 1;

    public static final double kPitchBotShelfMod = 1;
    public static final double kReachBotShelfMod = 1;

    public static final double kRetractedPitch = 1;
    public static final double kRetractedReach = 1;

    public static final double kScoreRotationAngle = 1;
    // cone top shelf grab

    // scoring constants

  }

  public static final class OIConstants {

    public static final int kDriverControllerPort = 0;
  }

  public static final class AutoConstants {

    // 4765: Not using these in autonomous yet.
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared =
      Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
      kMaxAngularSpeedRadiansPerSecond,
      kMaxAngularSpeedRadiansPerSecondSquared
    );
  }
}
