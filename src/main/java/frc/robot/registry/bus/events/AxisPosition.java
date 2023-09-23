package frc.robot.registry.bus.events;

import frc.robot.config.util.classes.Axis;
import frc.robot.registry.bus.main.Event;

public class AxisPosition extends Event {

  public double position;
  public Axis axis;

  public AxisPosition(double position, Axis axis) {
    this.position = position;
    this.axis = axis;
  }
}
