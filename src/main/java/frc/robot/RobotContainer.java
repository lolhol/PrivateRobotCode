// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutoBumpLeaveBalance;
import frc.robot.config.ControllerButtons;
import frc.robot.config.util.classes.Button;
import frc.robot.config.util.enums.Buttons;
import frc.robot.registry.bus.events.MillisecondEvent;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SubsystemTest;
import java.time.Duration;
import java.time.LocalDateTime;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  final SubsystemTest test = new SubsystemTest();
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private final ClawSubsystem m_clawSubsystem = new ClawSubsystem();

  // The driver's controller
  // 4765: converted this from xbox to joystick
  Joystick m_driverController = new Joystick(
    Constants.I_CONSTANTS.kDriverControllerPort
  );

  // Arm Controller
  XboxController m_armController = new XboxController(0);
  ControllerButtons buttons = new ControllerButtons(0, "test");

  // 4765: converted this from xbox to joystick
  // XboxController m_driverController = new
  // XboxController(OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureButtonBindings();

    buttons.addButton(new Button(2, Buttons.A));
    test.setDefaultCommand(new RunCommand(() -> test.tick(this.buttons), test));

    m_robotDrive.setDefaultCommand(
      new RunCommand(
        () ->
          m_robotDrive.drive(
            m_driverController.getRawAxis(1) * -1,
            m_driverController.getRawAxis(0) * -1,
            m_driverController.getRawAxis(2) * -1,
            false,
            m_driverController.getRawButton(6),
            m_driverController.getRawButton(3)
          ),
        m_robotDrive
      )
    );

    m_armSubsystem.setDefaultCommand(
      new RunCommand(
        () ->
          m_armSubsystem.move(
            m_armController.getRawAxis(1),
            m_armController.getRawAxis(3)
          ),
        m_armSubsystem
      )
    );

    m_clawSubsystem.setDefaultCommand(
      new RunCommand(
        () -> m_clawSubsystem.moveClaw(m_armController.getRawButton(6)),
        m_clawSubsystem
      )
    );
    // Registry.COMMAND_REGISTRY.regCommand(new ExampleCommand());
    // Register a sample event
    // Registry.EVENT_BUS.POST(new CustomEvent(true));
    // Init Millisecond Event
    // Registry.EVENT_BUS.register(new EventExample());
    // initMS();
  }

  void initMS() {
    // this will allow you to do something by ms and not normal ticks

    LocalDateTime now = LocalDateTime.now();
    Duration initialDelay = Duration.between(now, now);
    long initialDelaySeconds = initialDelay.getSeconds();

    ScheduledExecutorService threadPool = Executors.newScheduledThreadPool(1);
    threadPool.scheduleAtFixedRate(
      () -> Constants.REGISTRY.EVENT_BUS.post(new MillisecondEvent()),
      initialDelaySeconds,
      1,
      TimeUnit.MILLISECONDS
    );
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */

  private void configureButtonBindings() {
    // 4765: To be used to assign buttons for things like using the arm and the claw

    // m_armController.a().onTrue(new ClawGrab());

    // Trigger claw_trigger = new JoystickButton(m_armController, 1);
    // claw_trigger.onTrue(m_clawSubsystem.grab());

    // new JoystickButton(m_armController, 1)
    // .onFalse(new OpenClaw());

  }

  // 4765: Commented out the autonomous code that because it uses odometry which
  // we don't need yet

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //return new AutoBumpScore(m_robotDrive);

    //return new AutonDropBackUp(m_robotDrive, m_armSubsystem);

    return new AutoBumpLeaveBalance(m_robotDrive);
    //return new AutoBumpLeaveAutoBalance(m_robotDrive);

  }
  // // Create config for trajectory
  // TrajectoryConfig config =
  // new TrajectoryConfig(
  // AutoConstants.kMaxSpeedMetersPerSecond,
  // AutoConstants.kMaxAccelerationMetersPerSecondSquared)
  // // Add kinematics to ensure max speed is actually obeyed
  // .setKinematics(DriveConstants.kDriveKinematics);

  // // An example trajectory to follow. All units in meters.
  // Trajectory exampleTrajectory =
  // TrajectoryGenerator.generateTrajectory(
  // // Start at the origin facing the +X direction
  // new Pose2d(0, 0, new Rotation2d(0)),
  // // Pass through these two interior waypoints, making an 's' curve path
  // List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
  // // End 3 meters straight ahead of where we started, facing forward
  // new Pose2d(3, 0, new Rotation2d(0)),
  // config);

  // var thetaController =
  // new ProfiledPIDController(
  // AutoConstants.kPThetaController, 0, 0,
  // AutoConstants.kThetaControllerConstraints);
  // thetaController.enableContinuousInput(-Math.PI, Math.PI);

  // SwerveControllerCommand swerveControllerCommand =
  // new SwerveControllerCommand(
  // exampleTrajectory,
  // m_robotDrive::getPose, // Functional interface to feed supplier
  // DriveConstants.kDriveKinematics,

  // // Position controllers
  // new PIDController(AutoConstants.kPXController, 0, 0),
  // new PIDController(AutoConstants.kPYController, 0, 0),
  // thetaController,
  // m_robotDrive::setModuleStates,
  // m_robotDrive);

  // // Reset odometry to the starting pose of the trajectory.
  // m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

  // // Run path following command, then stop at the end.
  // return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0,
  // false));
  // }
}
