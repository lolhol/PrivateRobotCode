package frc.robot.config.util.enums;

public enum Buttons {
  LeftBumper,
  RightBumper,
  LeftStick,
  RightStick,
  A,
  B,
  X,
  Y,
  Back,
  Start;

  @Override
  public String toString() {
    var name = this.name();

    if (name.endsWith("Bumper")) {
      return name;
    }

    return name + "Button";
  }
}
