package frc.robot.subsystems;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PathfindThenFollowPathHolonomic;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotMap.DriveMap;
import frc.robot.core.Swerve.Swerve;
import frc.robot.core.Swerve.SwerveConstants;

import java.util.function.Consumer;
import java.util.function.Supplier;

import static com.pathplanner.lib.path.PathPlannerPath.fromPathFile;
import static edu.wpi.first.math.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds;
import static frc.robot.core.Swerve.SwerveConstants.*;

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

  public Command goToPoint(Pose2d targetPose) {
    PIDController xController = new PIDController(DRIVE_KP, DRIVE_KI, DRIVE_KD);
    PIDController yController = new PIDController(DRIVE_KP, DRIVE_KI, DRIVE_KD);
    PIDController rController = new PIDController(DRIVE_KP, DRIVE_KI, DRIVE_KD);
    return new FunctionalCommand(
            () -> System.out.println(String.format("Traveling to x:%s, y:%s, z:%s", targetPose.getX(), targetPose.getY(), targetPose.getRotation().getDegrees())),
            () -> {
                double sX = xController.calculate(getPose().getX(), targetPose.getX());
                double sY = yController.calculate(getPose().getY(), targetPose.getY());
                double sR = rController.calculate(getPose().getRotation().getRadians(), targetPose.getRotation().getRadians());

                drive(ChassisSpeeds.fromFieldRelativeSpeeds(sX, sY, sR, getPose().getRotation()), true);
            },
            interrupted -> { xController.close(); yController.close(); rController.close(); },
            () -> /*getPose().getTranslation().getDistance(targetPose.getTranslation()) < 0.1*/
                    xController.atSetpoint() && yController.atSetpoint() && rController.atSetpoint(),
            this
    );
  }

  /*public Command followTrajectoryCommand(String path, boolean isFirstPath) {
    PathPlannerTrajectory traj = loadPath(path, SwerveConstants.MAX_VELOCITY, SwerveConstants.MAX_ACCELERATION);
    return new FollowPathWithEvents(
            new FollowPathCommand(followTrajectoryCommand(traj, isFirstPath)), traj.get(), () -> this.getPose()
    );
  }*/

  private Command followTrajectoryCommand(Supplier<PathPlannerPath> traj, boolean isFirstPath) {
      // Create PIDControllers for each movement (and set default values)
      PIDConstants translationConstants = new PIDConstants(5.0, 0.0, 0.0);
      PIDConstants rotationConstants = new PIDConstants(1.0, 0.0, 0.0);

      PathFollowingController dc = new PPHolonomicDriveController(translationConstants, rotationConstants, 0.0, 0.0);
      Supplier<ChassisSpeeds> csS = KINEMATICS::toChassisSpeeds;
      Consumer<ChassisSpeeds> csC = ChassisSpeeds -> drive(ChassisSpeeds, true);

      return new FollowPathCommand(traj.get(), this::getPose, csS, csC, dc, new ReplanningConfig(), this);
    }

  private Command followTrajectoryCommand(String pathFile, boolean isFirstPath) {
          PathPlannerPath path = fromPathFile(pathFile);
          return new FollowPathWithEvents(followTrajectoryCommand(()->path, isFirstPath), path, this::getPose);
  }
}
