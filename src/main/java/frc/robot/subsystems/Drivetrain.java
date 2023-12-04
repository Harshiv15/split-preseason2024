package frc.robot.subsystems;

import frc.robot.RobotMap.DriveMap;
import frc.robot.core.Swerve.Swerve;

public class Drivetrain extends Swerve {
  private static Drivetrain instance;

  public static Drivetrain getInstance() {
    if (instance == null) instance = new Drivetrain();
    return instance;
  }

  private Drivetrain() {
    super(
        DriveMap.PIGEON_ID,
        DriveMap.FrontLeft.CONSTANTS,
        DriveMap.FrontRight.CONSTANTS,
        DriveMap.BackLeft.CONSTANTS,
        DriveMap.BackRight.CONSTANTS);
  }

  @Override
  public void periodic() {
    super.periodic();
  }
}
