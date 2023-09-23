package frc.robot.registry.bus.events;

import frc.robot.config.util.classes.Button;
import frc.robot.registry.bus.main.Event;

public class KeyPressEvent extends Event {

  public Button button;
  public boolean buttonState;

  public KeyPressEvent(Button button, boolean buttonState) {
    this.button = button;
    this.buttonState = buttonState;
  }
}
