// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

  // Robot swerve modules
  // 4765: Updated to our Port config:
  // one tunring enc
  // no separate address for driving encoder
  // magnet offset for each turning encoder
  // label for so shuffleboard can show each module separately
  final SwerveModule m_frontLeft = new SwerveModule(
    Constants.DRIVE.kFrontLeftDriveMotorPort,
    Constants.DRIVE.kFrontLeftTurningMotorPort,
    Constants.DRIVE.kFrontLeftTurningEncoderPort,
    Constants.DRIVE.kFrontLeftDriveEncoderReversed,
    Constants.DRIVE.kFrontLeftTurningEncoderReversed,
    Constants.DRIVE.kFrontLeftTurningMagnetOffset,
    "FL"
  );

  final SwerveModule m_rearLeft = new SwerveModule(
    Constants.DRIVE.kRearLeftDriveMotorPort,
    Constants.DRIVE.kRearLeftTurningMotorPort,
    Constants.DRIVE.kRearLeftTurningEncoderPort,
    Constants.DRIVE.kRearLeftDriveEncoderReversed,
    Constants.DRIVE.kRearLeftTurningEncoderReversed,
    Constants.DRIVE.kRearLeftTurningMagnetOffset,
    "RL"
  );

  final SwerveModule m_frontRight = new SwerveModule(
    Constants.DRIVE.kFrontRightDriveMotorPort,
    Constants.DRIVE.kFrontRightTurningMotorPort,
    Constants.DRIVE.kFrontRightTurningEncoderPort,
    Constants.DRIVE.kFrontRightDriveEncoderReversed,
    Constants.DRIVE.kFrontRightTurningEncoderReversed,
    Constants.DRIVE.kFrontRightTurningMagnetOffset,
    "FR"
  );

  final SwerveModule m_rearRight = new SwerveModule(
    Constants.DRIVE.kRearRightDriveMotorPort,
    Constants.DRIVE.kRearRightTurningMotorPort,
    Constants.DRIVE.kRearRightTurningEncoderPort,
    Constants.DRIVE.kRearRightDriveEncoderReversed,
    Constants.DRIVE.kRearRightTurningEncoderReversed,
    Constants.DRIVE.kRearRightTurningMagnetOffset,
    "RR"
  );

  // The gyro sensor

  // 4765 TODO: This currently throws an error at runtime. Gyro broken? Use NavX instead?
  private final Gyro m_gyro = new ADXRS450_Gyro();

  private final AHRS ahrs = new AHRS(I2C.Port.kMXP);

  // 4765: Commented out since not using odometry yet

  // // Odometry class for tracking robot pose
  // SwerveDriveOdometry m_odometry =
  // new SwerveDriveOdometry(
  // Constants.DRIVE.kDriveKinematics,
  // m_gyro.getRotation2d(),
  // new SwerveModulePosition[] {
  // m_frontLeft.getPosition(),
  // m_frontRight.getPosition(),
  // m_rearLeft.getPosition(),
  // m_rearRight.getPosition()
  // });

  // 4765: Added class variable for each deadband
  static final double xSpeedDeadband = Constants.DRIVE.kXSpeedDeadband;
  static final double ySpeedDeadband = Constants.DRIVE.kYSpeedDeadband;
  static final double rotDeadband = Constants.DRIVE.kRotDeadband;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {}

  @Override
  public void periodic() {
    // 4765: Commented out since not using odometry yet

    // Update the odometry in the periodic block
    // m_odometry.update(
    // m_gyro.getRotation2d(),
    // new SwerveModulePosition[] {
    // m_frontLeft.getPosition(),
    // m_frontRight.getPosition(),
    // m_rearLeft.getPosition(),
    // m_rearRight.getPosition()
    // });
  }

  // 4765: Commented out since not using odometry yet

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  // public Pose2d getPose() {
  // return m_odometry.getPoseMeters();
  // }

  // 4765: Commented out since not using odometry yet

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  // public void resetOdometry(Pose2d pose) {
  // m_odometry.resetPosition(
  // m_gyro.getRotation2d(),
  // new SwerveModulePosition[] {
  // m_frontLeft.getPosition(),
  // m_frontRight.getPosition(),
  // m_rearLeft.getPosition(),
  // m_rearRight.getPosition()
  // },
  // pose);
  // }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(
    double xSpeed,
    double ySpeed,
    double rot,
    boolean fieldRelative
  ) {
    SmartDashboard.putNumber("IMU_Yaw", ahrs.getYaw());
    SmartDashboard.putNumber("IMU_Pitch", ahrs.getPitch());
    SmartDashboard.putNumber("IMU_Roll", ahrs.getRoll());

    SmartDashboard.putNumber("IMU_CompassHeading", ahrs.getCompassHeading());

    // 4765: Puts joystick inout data on main shuffleboard for debugging
    SmartDashboard.putNumber("xSpeed", xSpeed);
    SmartDashboard.putNumber("ySpeed", ySpeed);
    SmartDashboard.putNumber("rot", rot);

    // 4765: Added a very (too?) basic deadband strategy
    if (Math.abs(xSpeed) < xSpeedDeadband) {
      xSpeed = 0;
    }
    if (Math.abs(ySpeed) < ySpeedDeadband) {
      ySpeed = 0;
    }
    if (Math.abs(rot) < rotDeadband) {
      rot = 0;
    }

    var swerveModuleStates =
      Constants.DRIVE.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
          ? ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed,
            ySpeed,
            rot,
            (Rotation2d.fromDegrees(ahrs.getYaw()))
          )
          : new ChassisSpeeds(xSpeed, ySpeed, rot)
      );

    SwerveDriveKinematics.desaturateWheelSpeeds(
      swerveModuleStates,
      Constants.DRIVE.kMaxSpeedMetersPerSecond
    );

    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public void drive(
    double xSpeed,
    double ySpeed,
    double rot,
    boolean fieldRelative,
    boolean throttled,
    boolean locked
  ) {
    SmartDashboard.putNumber("IMU_Yaw", ahrs.getYaw());
    SmartDashboard.putNumber("IMU_Pitch", ahrs.getPitch());
    SmartDashboard.putNumber("IMU_Roll", ahrs.getRoll());

    SmartDashboard.putNumber("IMU_CompassHeading", ahrs.getCompassHeading());

    // 4765: Puts joystick inout data on main shuffleboard for debugging
    SmartDashboard.putNumber("xSpeed", xSpeed);
    SmartDashboard.putNumber("ySpeed", ySpeed);
    SmartDashboard.putNumber("rot", rot);

    // 4765: Added a very (too?) basic deadband strategy
    if (Math.abs(xSpeed) < xSpeedDeadband) {
      xSpeed = 0;
    }
    if (Math.abs(ySpeed) < ySpeedDeadband) {
      ySpeed = 0;
    }
    if (Math.abs(rot) < rotDeadband) {
      rot = 0;
    }

    if (throttled) {
      xSpeed = xSpeed * 0.25;
      ySpeed = ySpeed * 0.25;
      rot = rot * 0.25;
    }

    if (locked) {
      m_frontLeft.setDesiredState(
        new SwerveModuleState(0.0, new Rotation2d(Math.PI / 4))
      );
      m_frontRight.setDesiredState(
        new SwerveModuleState(0.0, new Rotation2d(Math.PI / -4))
      );
      m_rearLeft.setDesiredState(
        new SwerveModuleState(0.0, new Rotation2d(Math.PI / -4))
      );
      m_rearRight.setDesiredState(
        new SwerveModuleState(0.0, new Rotation2d(Math.PI / 4))
      );
    } else {
      var swerveModuleStates =
        Constants.DRIVE.kDriveKinematics.toSwerveModuleStates(
          fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
              xSpeed,
              ySpeed,
              rot,
              (Rotation2d.fromDegrees(ahrs.getYaw()))
            )
            : new ChassisSpeeds(xSpeed, ySpeed, rot)
        );

      SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates,
        Constants.DRIVE.kMaxSpeedMetersPerSecond
      );

      m_frontLeft.setDesiredState(swerveModuleStates[0]);
      m_frontRight.setDesiredState(swerveModuleStates[1]);
      m_rearLeft.setDesiredState(swerveModuleStates[2]);
      m_rearRight.setDesiredState(swerveModuleStates[3]);
    }
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
      desiredStates,
      Constants.DRIVE.kMaxSpeedMetersPerSecond
    );
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  // 4765: Our encoders do not reset and this seems to be for odometry

  /** Resets the drive encoders to currently read a position of 0. */
  // public void resetEncoders() {
  // m_frontLeft.resetEncoders();
  // m_rearLeft.resetEncoders();
  // m_frontRight.resetEncoders();
  // m_rearRight.resetEncoders();
  // }

  // 4765 TODO: Figure out if we need any of these three functions for field
  // (driver) centric driving

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (Constants.DRIVE.kGyroReversed ? -1.0 : 1.0);
  }
}
