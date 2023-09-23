package frc.robot.config.util.enums;

public enum Axises {
  LeftX,
  RightX,
  LeftY,
  RightY,
  LeftTrigger,
  RightTrigger;

  @Override
  public String toString() {
    var name = this.name();
    if (name.endsWith("Trigger")) {
      return name + "Axis";
    }
    return name;
  }
}
