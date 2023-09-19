// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.registry.Registry;
import frc.robot.registry.bus.events.CustomEvent;
import frc.robot.registry.bus.events.MillisecondEvent;
import frc.robot.registry.bus.events.Tick;
import frc.robot.registry.bus.markers.SubscribeEvent;

public class SubsystemTest extends SubsystemBase {

  //private final DutyCycleEncoder m_reachEncoder = new DutyCycleEncoder(0);
  int count = 0;

  @SubscribeEvent
  public void msEvent(MillisecondEvent event) {
    if (count >= 1000) {
      count = 0;
      System.out.println("One second passed!");
      //Registry.EVENT_BUS.POST(new CustomEvent(false));
    }

    //System.out.println(event.curTime);

    count++;
  }

  @SubscribeEvent
  public void customEvent(CustomEvent event) {
    // System.out.println("Custom Event!");
  }
}
