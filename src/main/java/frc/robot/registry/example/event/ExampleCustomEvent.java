package frc.robot.registry.example.event;

import frc.robot.registry.Registry;
import frc.robot.registry.bus.main.Event;

// Make sure that your custom event class extends Event.java
// since that is what tells the code that this is a custom event class
// In the future I will implement features like cancelable etc. and Event
// Will be used for that
public class ExampleCustomEvent extends Event {

  public boolean variable;

  // You do not have to put anything in here but...
  public ExampleCustomEvent(boolean youCanChangeVariables) {
    this.variable = youCanChangeVariables;
  }

  // you can also put functions that will be accessable in the event function
  public boolean isVariableValid(boolean var) {
    return !var;
  }
}
