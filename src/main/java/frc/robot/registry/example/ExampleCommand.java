package frc.robot.registry.example;

import frc.robot.registry.Registry;
import frc.robot.registry.bus.events.CustomEvent;
import frc.robot.registry.command.main.Command;
import frc.robot.registry.command.main.DefaultHandler;

public class ExampleCommand extends Command {

  // You can put however many variables you want. They do not matter
  // If i see you putting 100000 variables here... this defeats the whole purpose
  boolean example = true;

  @Override
  public boolean shouldRun() {
    // The handle() will be called when @this returns true.
    return example;
  }

  @DefaultHandler(trueF = true)
  public void handle() {
    System.out.println(this.isAlreadyExecuted());
    // Do when it triggers.
    // Registry.EVENT_BUS.POST(new CustomEvent(getValue()));
  }

  // You can add as many functions at you want. BUT I HIGHLY RECOMMEND
  // THAT YOU DO NOT!
  public boolean getValue() {
    return !example;
  }
}
// THERE SHOULD BE 2 FUNCTIONS IN THIS! MAX!
// NO MORE!
