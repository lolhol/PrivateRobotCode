package frc.robot.config.util.classes;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.config.util.enums.Axises;

public class Axis {

  int id;
  Axises label;
  String name;

  public Axis(int id, Axises label) {
    this.id = id;
    this.label = label;
    this.name = label.toString();
  }

  public int getId() {
    return id;
  }

  public Axises getLabel() {
    return label;
  }

  public double axisState(int port) {
    return DriverStation.getStickAxis(port, this.id);
  }
}
