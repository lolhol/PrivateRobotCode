package frc.robot.config;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.config.util.classes.Button;
import frc.robot.config.util.enums.Buttons;
import frc.robot.subsystems.util.file.JsonParser;
import java.io.File;
import java.util.HashMap;
import java.util.Map;

public class ControllerButtons {

  HashMap<Buttons, Button> buttons = new HashMap<>();

  int port;
  String name;
  XboxController controller;

  public ControllerButtons(int port, String name) {
    this.port = port;
    this.name = name;
    this.controller = new XboxController(port);
    // Hook @this up to a json file and check if "name" is there
  }

  public boolean addButton(Button button) {
    if (isExists(button.getLabel())) {
      return false;
    }

    buttons.put(button.getLabel(), button);
    return true;
  }

  public boolean getButtonState(Button button) {
    return button.buttonState(this.port);
  }

  public boolean getButtonState(Buttons button) {
    return (buttons.get(button).buttonState(this.port));
  }

  boolean isExists(Buttons button) {
    return buttons.containsKey(button);
  }

  public void addAllFromJsonFile(File file, String name) {
    JsonParser parser = new JsonParser();
    buttons.clear();

    for (Map.Entry<String, Integer> entry : parser
      .getMap(parser.parseData(file), name)
      .entrySet()) {}
  }
}
