// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.ColorSensorV3.Register;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.registry.Registry;
import frc.robot.registry.bus.main.EventBus;
import frc.robot.registry.command.main.CommandRegistry;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public final static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // This cuts down basically the constants to only 8 instead of 1000+

  static final Constants CONST = new Constants();

  public static final Registry REGISTRY = new Registry();
  public static final DriveConstants DRIVE = CONST.new DriveConstants();
  public static final ModuleConstants MODULE = CONST.new ModuleConstants();
  public static final ArmConstants ARM = CONST.new ArmConstants();
  public static final OIConstants I_CONSTANTS = CONST.new OIConstants();
  public static final AutoConstants AUTO = CONST.new AutoConstants();

  public final class DriveConstants {

    // 4765: Used our addresses
    public final int kFrontLeftDriveMotorPort = 10;
    public final int kRearLeftDriveMotorPort = 21;
    public final int kFrontRightDriveMotorPort = 12;
    public final int kRearRightDriveMotorPort = 13;

    public final int kFrontLeftTurningMotorPort = 22;
    public final int kRearLeftTurningMotorPort = 25;
    public final int kFrontRightTurningMotorPort = 20;
    public final int kRearRightTurningMotorPort = 23;

    public final int kFrontLeftTurningEncoderPort = 1;
    public final int kRearLeftTurningEncoderPort = 4;
    public final int kFrontRightTurningEncoderPort = 2;
    public final int kRearRightTurningEncoderPort = 3;

    // 4765: determined by trial and error - probably right?

    public final boolean kFrontLeftTurningEncoderReversed = false;
    public final boolean kRearLeftTurningEncoderReversed = false;
    public final boolean kFrontRightTurningEncoderReversed = false;
    public final boolean kRearRightTurningEncoderReversed = false;

    // 4765: determined by trial and error - probably right?

    public final boolean kFrontLeftDriveEncoderReversed = false;
    public final boolean kRearLeftDriveEncoderReversed = false;
    public final boolean kFrontRightDriveEncoderReversed = false;
    public final boolean kRearRightDriveEncoderReversed = false;

    // 4765: figured out by using Phoenix Tuner app

    // Final(?) calibration with arm in final(?) orientation
    public final double kFrontLeftTurningMagnetOffset = 21.7;
    public final double kRearLeftTurningMagnetOffset = -88.0;
    public final double kFrontRightTurningMagnetOffset = 209.25;
    public final double kRearRightTurningMagnetOffset = -145.45;

    // less old
    // public final double kFrontLeftTurningMagnetOffset = -57.75;
    // public final double kRearLeftTurningMagnetOffset = -61.98;
    // public final double kFrontRightTurningMagnetOffset = 1.84;
    // public final double kRearRightTurningMagnetOffset = 111.11;

    // old
    // public final double kFrontLeftTurningMagnetOffset = -62.5;
    // public final double kRearLeftTurningMagnetOffset = 110.75;
    // public final double kFrontRightTurningMagnetOffset = -56.2;
    // public final double kRearRightTurningMagnetOffset = 1.75;

    // 4765: Measured by Justin!

    public final double kTrackWidth = 0.47625;
    // Distance between centers of right and left wheels on robot
    public final double kWheelBase = 0.47625;
    // Distance between front and back wheels on robot

    // 4765 NOTE: WPILIB coordinate system is reportedly: x forward, y left, z up,
    // CCW positive angles

    // 4765: If behavior is backwards/upside-down feeling, explore messing with
    // this. Seems right currently.
    public final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
    );

    // 4765 TODO: Need to fix and configure gyro or navex for field (driver) centric
    // driving
    public final boolean kGyroReversed = false;

    // 4765: commented out because I think it has to do with odometry

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or
    // theoretically
    // for *your* robot's drive.
    // The SysId tool provides a convenient method for obtaining these values for
    // your robot.
    // public final double ksVolts = 1;
    // public final double kvVoltSecondsPerMeter = 0.8;
    // public final double kaVoltSecondsSquaredPerMeter = 0.15;

    // 4765: Not sure what this should be configured to be, but made it slow for
    // safe testing during development
    // 4765 TODO: deteremine what this should be set to
    public final double kMaxSpeedMetersPerSecond = 4;

    // 4765: Introduced some deadband constants (remember that X is forward!)
    public final double kXSpeedDeadband = 0.05;
    public final double kYSpeedDeadband = 0.05;
    public final double kRotDeadband = 0.05;
  }

  public final class ModuleConstants {

    // 4765 TODO: determine what these should actuall be
    public final double kMaxModuleAngularSpeedRadiansPerSecond =
      2 * Math.PI * 24.1;
    public final double kMaxModuleAngularAccelerationRadiansPerSecondSquared =
      2 * Math.PI * 24.1;

    // 4765 TODO: determine what these should actually be
    // 4765 TODO: determine what effect this has (or is it just odometry)

    public final int kEncoderCPR = 4096;
    public final double kWheelDiameterMeters = 0.15;
    public final double kDriveEncoderDistancePerPulse =
      // Assumes the encoders are directly mounted on the wheel shafts
      (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

    // 4765 TODO: determine what effect this has (or is it just odometry)
    public final double kTurningEncoderDistancePerPulse =
      // Assumes the encoders are on a 1:1 reduction with the module shaft.
      (2 * Math.PI) / (double) kEncoderCPR;

    // 4765 TODO: Test and tune this. Important!!!

    // Drive PID P value
    public final double kPModuleDriveController = 0.8;

    // 4765 TODO: Test and tune this. Important!!!

    // Turning PID P value
    public final double kPModuleTurningController = 0.05;
  }

  public final class ArmConstants {

    public final double kPitchLevel = 0;
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

    public final double kPitchConeTopScoreMod = 1;
    public final double kPitchConeMidScoreMod = 1;
    public final double kPitchConeBotScoreMod = -1;

    public final double kPitchCubeTopScoreMod = 1;
    public final double kPitchCubeMidScoreMod = 1;
    public final double kPitchCubeBotScoreMod = -1;

    public final double kReachConeTopScoreMod = 1;
    public final double kReachConeMidScoreMod = 1;
    public final double kReachConeBotScoreMod = 1;

    public final double kReachCubeTopScoreMod = 1;
    public final double kReachCubeMidScoreMod = 1;
    public final double kReachCubeBotScoreMod = 1;

    public final double kPitchTopShelfMod = 1;
    public final double kReachTopShelfMod = 1;

    public final double kPitchBotShelfMod = 1;
    public final double kReachBotShelfMod = 1;

    public final double kRetractedPitch = 1;
    public final double kRetractedReach = 1;

    public final double kScoreRotationAngle = 1;
    // cone top shelf grab

    // scoring constants

  }

  public final class OIConstants {

    public final int kDriverControllerPort = 0;
    public final XboxController xBoxController = new XboxController(1);
  }

  public final class AutoConstants {

    // 4765: Not using these in autonomous yet.
    public final double kMaxSpeedMetersPerSecond = 3;
    public final double kMaxAccelerationMetersPerSecondSquared = 3;
    public final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public final double kPXController = 1;
    public final double kPYController = 1;
    public final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
      kMaxAngularSpeedRadiansPerSecond,
      kMaxAngularSpeedRadiansPerSecondSquared
    );
  }
}
