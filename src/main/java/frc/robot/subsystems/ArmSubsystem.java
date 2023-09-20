// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.util.arm.PitchSubsystem;
import frc.robot.subsystems.util.arm.ReachSubsystem;

public class ArmSubsystem extends SubsystemBase {

  final ReachSubsystem m_reachSubsystem = new ReachSubsystem();
  final PitchSubsystem m_pitchSubsystem = new PitchSubsystem();
  final ClawSubsystem m_clawSubsystem = new ClawSubsystem();

  final float coef = 0.9F;

  public void movePitch(double axis) {
    m_pitchSubsystem.move(axis);
  }

  public void moveReach(double axis) {
    m_reachSubsystem.move(axis);
  }

  //public void moveClaw(double axis){
  //    m_clawSubsystem.grab();
  //}

  public void setArm(double pitch, double reach) {
    m_pitchSubsystem.set(pitch);
    m_reachSubsystem.set(reach);
  }

  public void move(double pitch, double reach) {
    // TODO: Do smart things here...

    // ---------------------------------------------- //
    m_pitchSubsystem.move(pitch * coef);
    m_reachSubsystem.move(reach * coef);
    // ---------------------------------------------- //

    //if (grab > -1) {
    //    m_clawSubsystem.grab();
    //} else {
    //    m_clawSubsystem.release();
    //}
  }

  public double getPosPitch() {
    return m_pitchSubsystem.getPos();
  }

  public double getPosReach() {
    return m_reachSubsystem.getPos();
  }
}
