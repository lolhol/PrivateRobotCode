package frc.robot.registry;

import frc.robot.registry.bus.main.EventBus;
import frc.robot.registry.command.main.CommandRegistry;

public class Registry {

  public final EventBus EVENT_BUS = new EventBus();

  //Might be used in the future, made a framework so it is there just so u know :/
  public final CommandRegistry COMMAND_REGISTRY = new CommandRegistry();
}
