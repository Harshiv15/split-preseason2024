package frc.robot.auto.selector;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoModeSelector {
  private static AutoModeSelector instance;

  public static AutoModeSelector getInstance() {
    if (instance == null) {
      instance = new AutoModeSelector();
    }
    return instance;
  }

  // private final DoNothing doNothing = new DoNothing();

    private final SendableChooser<Command> modeChooserRed;
    private final SendableChooser<Command> modeChooserBlue;

  private AutoModeSelector() {
    modeChooserRed = new SendableChooser<>();
    modeChooserBlue = new SendableChooser<>();
    updateAutoModeSelector();
  }

  public void updateAutoModeSelector() {
  }
  public SendableChooser<Command> getRedChooser() {
    return modeChooserRed;
  }

  public SendableChooser<Command> getBlueChooser() {
    return modeChooserBlue;
  }
}
