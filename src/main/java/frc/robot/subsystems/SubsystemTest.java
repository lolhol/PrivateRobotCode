package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.registry.bus.events.AxisPosition;
import frc.robot.registry.bus.events.KeyPressEvent;
import frc.robot.registry.bus.markers.SubscribeEvent;

public class SubsystemTest extends SubsystemBase {

  public void tick(XboxController buttons) {
    //System.out.println(buttons.getLeftStickButton());
    System.out.println(buttons.getRawButton(1));
    //System.out.println(buttons.getBackButton());
    //System.out.println(buttons.getButtonState(Buttons.A));
  }

  @SubscribeEvent
  public void onAxisPosition(AxisPosition event) {
    System.out.println(event.position);
  }

  @SubscribeEvent
  public void onButtonPress(KeyPressEvent event) {
    System.out.println(event.button.getLabel().toString());
  }
}
