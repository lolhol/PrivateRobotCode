package frc.robot.config.util.classes;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.config.util.enums.Buttons;

public class Button {

  int id;
  Buttons label;
  String name;

  public Button(int id, Buttons label) {
    this.id = id;
    this.label = label;
    this.name = label.toString();
  }

  public int getId() {
    return id;
  }

  public Buttons getLabel() {
    return label;
  }

  public boolean buttonState(int port) {
    return DriverStation.getStickButton(port, (byte) this.id);
  }

  public boolean getPressed(int port) {
    return DriverStation.getStickButtonPressed(port, (byte) this.id);
  }

  public boolean getReleased(int port) {
    return DriverStation.getStickButtonPressed(port, (byte) this.id);
  }
}
