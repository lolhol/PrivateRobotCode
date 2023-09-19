package frc.robot.registry.bus.main;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.registry.bus.events.Tick;
import frc.robot.registry.bus.markers.SubscribeEvent;
import frc.robot.registry.bus.markers.SubscribeVar;
import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;

// WARNING! THIS IS { BETA } SO IF YOU ENCOUNTER [ANY] CRASHES REPORT THEM TO ME (godbrigero / lolhol / 26dkoterov@pinewood.edu)! z

// TEST!
// TODO: add a priority queue system (subscribe event)

public class EventBus {

  Map<SubsystemBase, EventClass> running = new HashMap<>();

  Map<String, HashSet<CustomEventClass>> runningCustom = new HashMap<>();
  Map<String, HashSet<VaribaleEventClass>> runningSubscribeVars = new HashMap<>();
  List<CustomEventClass> priorityOrder = new ArrayList<>();

  boolean isCanceled = false;

  public void register(SubsystemBase target) {
    if (target == null) {
      return;
    }

    EventClass eventClass = getEventClass(target);
    boolean isSuccessful = checkForCustom(target);
    boolean isVarFound = checkForVar(target);

    if (eventClass == null || !isSuccessful) {
      System.out.println(
        "The class with name -> " +
        target.getName() +
        " does not have a @SubscribeEvent over any functions."
      );

      return;
    }

    CommandScheduler
      .getInstance()
      .setDefaultCommand(target, eventClass.command);

    running.put(target, eventClass);
  }

  public boolean checkForVar(SubsystemBase target) {
    for (Field field : target.getClass().getFields()) {
      if (!field.isAnnotationPresent(SubscribeVar.class)) continue;

      SubscribeVar anotation = field.getAnnotation(SubscribeVar.class);
      if (
        runningSubscribeVars.containsKey(anotation.eventType()) &&
        runningSubscribeVars
          .get(anotation.eventType())
          .contains(getEventClass(field, target))
      ) continue;

      HashSet<VaribaleEventClass> previous = runningSubscribeVars.get(
        anotation.eventType()
      );

      previous.add(getEventClass(field, target));
      runningSubscribeVars.replace(
        anotation.eventType(),
        runningSubscribeVars.get(anotation.eventType()),
        previous
      );
    }

    return false;
  }

  public void post(String type) {}

  VaribaleEventClass getEventClass(Field field, SubsystemBase t) {
    try {
      VaribaleEventClass returnVar = new VaribaleEventClass(
        field.getAnnotation(SubscribeVar.class).endState(),
        field.get(t),
        t
      );
      return returnVar;
    } catch (IllegalArgumentException | IllegalAccessException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
      return null;
    }
  }

  // Will run the tick ahead of time
  public void run() {
    CommandScheduler.getInstance().run();
  }

  // This does not apply for ticks btw
  // This canceles ALL the events
  public void setCanceled(Boolean isCanceled) {
    this.isCanceled = isCanceled;
  }

  boolean checkForCustom(SubsystemBase target) {
    List<Method> customMethods = getCustomMethod(target);

    if (customMethods == null || customMethods.isEmpty()) return false;

    for (Method method : customMethods) {
      if (
        customMethods == null ||
        !Event.class.isAssignableFrom(method.getParameterTypes()[0]) ||
        Tick.class.isAssignableFrom(method.getParameterTypes()[0])
      ) continue;

      CustomEventClass custom = new CustomEventClass(target, method);
      String name = method.getParameterTypes()[0].getName();

      if (
        custom == null || custom.classDecl == null || custom.method == null
      ) continue;

      if (runningCustom.containsKey(name)) {
        if (!runningCustom.get(name).contains(custom)) {
          runningCustom.get(name).add(custom);
        }
      } else {
        HashSet<CustomEventClass> hash = new HashSet<>();
        hash.add(custom);

        runningCustom.put(name, hash);
      }
    }

    return true;
  }

  public void post(Event event) {
    if (isCanceled) return;

    String hashCode = event.getClass().getName();

    HashSet<CustomEventClass> customEvents = runningCustom.get(hashCode);
    for (CustomEventClass customEvent : customEvents) {
      try {
        customEvent.method.invoke(customEvent.classDecl, event);
      } catch (
        IllegalAccessException
        | IllegalArgumentException
        | InvocationTargetException e
      ) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      }
    }
  }

  // L8tr
  public void makePrimary(SubsystemBase target) {
    if (target == null) {
      return;
    }

    if (running.containsKey(target)) {
      EventClass eventClass = getEventClass(target);

      if (eventClass == null) {
        System.out.println(
          "The class with name -> " +
          target.getName() +
          " does not have a @SubscribeEvent over any functions."
        );

        return;
      }

      EventClass prev = running.get(target);
      running.replace(target, running.get(target), eventClass);

      unRegister(prev.base);
      register(eventClass.base);
    } else {
      register(target);
    }
  }

  public void unRegister(Subsystem target) {
    if (running.containsKey(target)) {
      running.remove(target);
    }

    for (Method method : target.getClass().getDeclaredMethods()) {
      if (
        method.isAnnotationPresent(SubscribeEvent.class) &&
        method.getParameterTypes().length > 0 &&
        method.getParameterTypes()[0] == Tick.class
      ) {
        CommandScheduler.getInstance().unregisterSubsystem(target);
      }
    }
  }

  private Method getMethod(SubsystemBase target) {
    for (Method method : target.getClass().getDeclaredMethods()) {
      if (
        method.isAnnotationPresent(SubscribeEvent.class) &&
        method.getParameterTypes().length > 0 &&
        method.getParameterTypes()[0] == Tick.class
      ) {
        return method;
      }
    }

    return null;
  }

  List<Method> getCustomMethod(SubsystemBase target) {
    List<Method> methods = new ArrayList<>();

    for (Method method : target.getClass().getDeclaredMethods()) {
      // ticks!

      if (
        method.isAnnotationPresent(SubscribeEvent.class) &&
        method.getParameterTypes().length > 0
      ) {
        methods.add(method);
      }
    }

    return methods;
  }

  EventClass getEventClass(SubsystemBase target) {
    if (target == null) {
      System.out.println("You passed a null in the register!");
      return null;
    }

    Method method = getMethod(target);

    if (method == null) {
      return null;
    }

    RunCommand command = new RunCommand(
      () -> {
        try {
          method.invoke(target, new Tick());
        } catch (
          IllegalAccessException
          | IllegalArgumentException
          | InvocationTargetException e
        ) {
          e.printStackTrace();
        }
      },
      target
    );

    return new EventClass(target, method, command);
  }

  class EventClass {

    SubsystemBase base;
    Method method;
    RunCommand command;

    public EventClass(SubsystemBase base, Method method, RunCommand command) {
      this.base = base;
      this.method = method;
      this.command = command;
    }
  }

  class CustomEventClass {

    SubsystemBase classDecl;
    Method method;
    Event eventInstance;

    public CustomEventClass(SubsystemBase classDecl, Method method) {
      this.classDecl = classDecl;
      this.method = method;
    }

    public int hashCode() {
      return method.hashCode();
    }

    public boolean equals(CustomEventClass other) {
      return hashCode() == other.hashCode();
    }
  }

  class VaribaleEventClass {

    Object stateOne;
    Object stateTwo;
    SubsystemBase base;

    public VaribaleEventClass(Object one, Object two, SubsystemBase base) {
      this.stateOne = one;
      this.stateTwo = two;
      this.base = base;
    }
  }
}
