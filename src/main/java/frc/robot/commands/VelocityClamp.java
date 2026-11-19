package frc.robot.commands;

import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.command2.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.swerve.Drive;
import frc.robot.subsystems.swerve.DriveConstants;

public class VelocityClamp extends Command {
  private final Drive drive;

  public VelocityClamp(Drive drive) {
    this.drive = drive;

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    DriveConstants.HOLONOMIC_DRIVE_CONTROLLER.reset(
        RobotState.getInstance().getEstimatedPose(), drive.getRobotSpeeds());
  }

  @Override
  public void execute() {
    Pose2d estimatedPose = RobotState.getInstance().getEstimatedPose();

    PathPlannerTrajectoryState targetState = new PathPlannerTrajectoryState();
    targetState.fieldSpeeds = new ChassisVelocities();
    targetState.pose = estimatedPose;

    drive.setTrajectorySpeeds(
        DriveConstants.HOLONOMIC_DRIVE_CONTROLLER.calculateRobotRelativeSpeeds(
            estimatedPose, targetState));
  }

  @Override
  public boolean isFinished() {
    ChassisVelocities robotSpeeds = drive.getRobotSpeeds();
    double speed = Math.hypot(robotSpeeds.vx, robotSpeeds.vy);
    return speed < 2;
  }
}
