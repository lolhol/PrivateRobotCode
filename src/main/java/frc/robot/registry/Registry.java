package frc.robot.registry;

import frc.robot.registry.bus.main.EventBus;
import frc.robot.registry.command.main.CommandRegistry;

public class Registry {

  public static final EventBus EVENT_BUS = new EventBus();
  public static final CommandRegistry COMMAND_REGISTRY = new CommandRegistry();
}
