package frc.robot.registry.command.main;

public abstract class Command {

  boolean isExcecuted = false;

  public boolean shouldRun() {
    return false;
  }

  public boolean isAlreadyExecuted() {
    return isExcecuted;
  }

  public void setExecuted(boolean state) {
    isExcecuted = state;
  }
}
