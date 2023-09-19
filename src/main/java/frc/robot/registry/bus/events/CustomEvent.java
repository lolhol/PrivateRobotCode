package frc.robot.registry.bus.events;

import frc.robot.registry.bus.main.Event;

// @this is an example!

public class CustomEvent extends Event {

  public boolean b = false;

  // By doing this, you can modify the variables inside the class
  public CustomEvent(boolean b) {
    this.b = b;
  }

  // You can make custom functions to be passed into event variable
  public boolean getB() {
    return b;
  }
  // Here you can make custom events as long as they extend an Event (will soon have a use)
}
