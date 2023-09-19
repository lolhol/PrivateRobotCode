package frc.robot.registry.example.event;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.registry.bus.events.MillisecondEvent;
import frc.robot.registry.bus.events.Tick;
import frc.robot.registry.bus.markers.SubscribeEvent;
import frc.robot.registry.bus.markers.SubscribeVar;

// Make sure that your class extends "SubsystemBase" since without it you can't
// really registrate the 25ms
// Might be fixed in the future (if it will be a big issue)
public class EventExample extends SubsystemBase { // This goes for custom and normal events

  @SubscribeVar(endState = true, eventType = "ExempleEvent")
  public boolean exampleEventBool = false;

  // Make sure that @SubscribeEvent is present!
  @SubscribeEvent
  public void tick(Tick event) { // See how I add the "Tick event" in params,
    // This is the thing that tells the program what
    // the difference between Tick event and CustomEvent is

    // ---------------------------------------------------- //

    // This will be called every 25ms
  }

  @SubscribeEvent
  public void somethingNew(Tick event) {
    // This WILL NOT BE CALLED!
    // If you have two ticks in one file, the closest to top
    // will be called and the bottom one will be ignored.

    // -------------------------------------------------
    // Some smart code here :)
    // -------------------------------------------------
  }

  @SubscribeEvent
  public void msEvent(MillisecondEvent event) {
    // This will be called every millisecond
    System.out.println(event.curTime);
    // You can pass some functions in the "event" variable via
    // Adding them to MillisecondEvent.java

    // ------------------------------------------------------

    // Functions can be called from inside the function thingy
    //someFunction();
  }

  public void someFunction() {
    return;
  }
}
