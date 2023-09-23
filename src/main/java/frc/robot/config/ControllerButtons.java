package frc.robot.config;

import com.google.gson.JsonObject;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.config.util.classes.Axis;
import frc.robot.config.util.classes.Button;
import frc.robot.config.util.enums.Axises;
import frc.robot.config.util.enums.Buttons;
import frc.robot.registry.bus.events.AxisPosition;
import frc.robot.registry.bus.events.KeyPressEvent;
import frc.robot.registry.bus.events.Tick;
import frc.robot.registry.bus.markers.SubscribeEvent;
import frc.robot.subsystems.util.file.JsonParser;
import java.io.File;
import java.util.HashMap;
import java.util.Map;

public class ControllerButtons extends SubsystemBase {

  HashMap<Buttons, Button> buttons = new HashMap<>();
  HashMap<Button, Boolean> buttonRegistry = new HashMap<>();

  HashMap<Axises, Axis> axis = new HashMap<>();
  HashMap<Axis, Double> axisRegistry = new HashMap<>();

  double previousState = -1;

  int port;
  String name;
  XboxController controller;

  public ControllerButtons(int port, String name) {
    this.port = port;
    this.name = name;
    this.controller = new XboxController(port);

    File configFile = new File(
      "src/main/java/frc/robot/config/util/data/data.json"
    );

    this.addAllButtonsFromJsonFile(configFile, name);
    this.addAllAxisFromJsonFile(configFile, name);

    Constants.REGISTRY.EVENT_BUS.register(this);
    // Hook @this up to a json file and check if "name" is there
  }

  public boolean addButton(Button button) {
    if (isExists(button.getLabel())) {
      return false;
    }

    buttons.put(button.getLabel(), button);
    buttonRegistry.put(button, false);
    return true;
  }

  public boolean addAxis(Axis axis) {
    if (this.axis.containsKey(axis.getLabel())) {
      return false;
    }

    this.axis.put(axis.getLabel(), axis);
    this.axisRegistry.put(axis, axis.axisState(this.port));
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

  public void addAllButtonsFromJsonFile(File file, String name) {
    JsonParser parser = new JsonParser();
    JsonObject obj = parser.parseData(file);
    Map<Buttons, Integer> map = parser.getMapButton(obj, name, "Buttons");

    for (Map.Entry<Buttons, Integer> entry : map.entrySet()) {
      Button button = new Button(entry.getValue(), entry.getKey());
      addButton(button);
    }
  }

  public void addAllAxisFromJsonFile(File file, String name) {
    JsonParser parser = new JsonParser();
    JsonObject obj = parser.parseData(file);
    Map<Axises, Integer> map = parser.getMapAxis(obj, name, "Axis");

    for (Map.Entry<Axises, Integer> entry : map.entrySet()) {
      Axis axis = new Axis(entry.getValue(), entry.getKey());
      addAxis(axis);
    }
  }

  @SubscribeEvent
  public void onTick(Tick event) {
    for (Map.Entry<Axis, Double> axis : axisRegistry.entrySet()) {
      double axisPos = axis.getKey().axisState(this.port);

      if (Math.abs(axisPos - axis.getValue()) * 100 > 3) {
        Constants.REGISTRY.EVENT_BUS.post(
          new AxisPosition(axisPos, axis.getKey())
        );

        axisRegistry.replace(axis.getKey(), axisPos);
      }
    }

    for (Map.Entry<Button, Boolean> button : buttonRegistry.entrySet()) {
      boolean buttonState = button.getKey().buttonState(this.port);
      boolean isReg = button.getValue();

      if (isReg && !buttonState) {
        buttonRegistry.replace(button.getKey(), false);
      }

      if (buttonState && !isReg) {
        Constants.REGISTRY.EVENT_BUS.post(
          new KeyPressEvent(button.getKey(), true)
        );

        buttonRegistry.replace(button.getKey(), true);
      }
    }
  }
}
