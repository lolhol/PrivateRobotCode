// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;

/** An example command that uses an example subsystem. */
public class ScoreTopLeft extends SequentialCommandGroup {

  /**
   * Creates a new ExampleCommand.
   *
   *
   */
  public ScoreTopLeft(DriveSubsystem m_drive, ArmSubsystem m_arm) {
    addCommands(
      new AlignWithTag(m_drive),
      new RotateAmount(m_drive, Constants.ARM.kScoreRotationAngle),
      new DriveToScore(m_drive),
      new RotateAmount(m_drive, -Constants.ARM.kScoreRotationAngle),
      new SetArm(
        m_arm,
        Constants.ARM.kPitchConeTopScoreMod,
        Constants.ARM.kReachConeTopScoreMod
      ),
      new OpenClaw(),
      new RetractArm(m_arm)
    );
  }
  // Called once the command ends or is interrupted.
}
