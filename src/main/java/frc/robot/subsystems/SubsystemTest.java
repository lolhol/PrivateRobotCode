package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.ControllerButtons;
import frc.robot.registry.bus.events.KeyPressEvent;
import frc.robot.registry.bus.markers.SubscribeEvent;

public class SubsystemTest extends SubsystemBase {

  public void tick(ControllerButtons buttons) {
    //System.out.println(buttons.getRawButton(1));
    //System.out.println(buttons.getButtonState(Buttons.A));
  }

  @SubscribeEvent
  public void onButtonClick(KeyPressEvent event) {
    System.out.println(event.buttonState);
  }
}
