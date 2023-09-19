package frc.robot.registry.command.main;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.registry.Registry;
import frc.robot.registry.bus.events.Tick;
import frc.robot.registry.bus.markers.SubscribeEvent;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;

// WARNING! THIS IS { BETA } SO IF YOU ENCOUNTER [ANY] CRASHES REPORT THEM TO ME (godbrigero / lolhol / 26dkoterov@pinewood.edu)!

public class CommandRegistry extends SubsystemBase {

  List<ListClass> commands = new ArrayList<>();

  public void registerCommands() {
    Registry.EVENT_BUS.register(this);
  }

  public void regCommand(Command command) {
    Method method = getMethod(command);

    if (command != null && method != null) {
      commands.add(new ListClass(command, method));
    }
  }

  private Method getMethod(Command target) {
    for (Method method : target.getClass().getDeclaredMethods()) {
      if (method.isAnnotationPresent(DefaultHandler.class)) {
        return method;
      }
    }

    return null;
  }

  // Ik this is slow, if u tell me how to make faster... then W :)
  @SubscribeEvent
  public void tick(Tick event) {
    for (ListClass command : commands) {
      if (command.command.shouldRun()) {
        if (!command.command.isExcecuted) {
          try {
            command.method.invoke(command.command);
            command.command.setExecuted(true);
          } catch (
            IllegalAccessException
            | IllegalArgumentException
            | InvocationTargetException e
          ) {
            e.printStackTrace();
          }
        }
      } else {
        if (command.command.isExcecuted) command.command.setExecuted(true);
      }
    }
  }

  public class ListClass {

    public Command command;
    public Method method;

    public ListClass(Command command, Method method) {
      this.command = command;
      this.method = method;
    }
  }
}
