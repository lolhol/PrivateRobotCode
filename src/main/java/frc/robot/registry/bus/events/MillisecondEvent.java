package frc.robot.registry.bus.events;

import frc.robot.registry.bus.main.Event;

public class MillisecondEvent extends Event {

  public long curTime = 0;

  public MillisecondEvent() {
    curTime = System.currentTimeMillis();
  }
}
