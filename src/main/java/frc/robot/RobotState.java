// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.FlippingUtil;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.util.Nat;
import org.wpilib.math.estimator.SwerveDrivePoseEstimator;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.math.util.Units;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchType;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.framework.RobotBase;
import org.wpilib.command2.Command;
import frc.robot.subsystems.swerve.DriveConstants;
import frc.robot.subsystems.vision.VisionConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/* based on wpimath/../PoseEstimator.java */
public class RobotState {
  public static final double fieldSizeX = Units.feetToMeters(57.573);
  public static final double fieldSizeY = Units.feetToMeters(26.417);

  public record OdometryMeasurement(
      SwerveModulePosition[] wheelPositions, Rotation2d gyroAngle, double timestamp) {}

  public record VisionMeasurement(Pose2d visionPose, double timestamp) {}

  private static final double poseBufferSizeSeconds = 2; // shorter?
  private static final Pose2d initialPose =
      isAllianceRed()
          ? FlippingUtil.flipFieldPose(DriveConstants.INITIAL_POSE)
          : DriveConstants.INITIAL_POSE;

  private final Matrix<N3, N1> matrixQ = new Matrix<>(Nat.N3(), Nat.N1());

  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(
          DriveConstants.KINEMATICS,
          new Rotation2d(),
          new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
          },
          initialPose,
          DriveConstants.STATE_STD_DEVS,
          VisionConstants.VISION_STATE_STD_DEVS);

  private Pose2d estimatedPose = initialPose; // vision adjusted

  private Pose2d lastApproachPose = new Pose2d();

  private ChassisVelocities robotSpeeds = new ChassisVelocities();

  private static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null) instance = new RobotState();
    return instance;
  }

  private RobotState() {
    for (int i = 0; i < 3; ++i) {
      matrixQ.set(
          i, 0, DriveConstants.STATE_STD_DEVS.get(i, 0) * DriveConstants.STATE_STD_DEVS.get(i, 0));
    }
  }

  /* update pose estimation based on odometry measurements*/
  public void addOdometryMeasurement(OdometryMeasurement measurement) {
    poseEstimator.updateWithTime(
        measurement.timestamp(), measurement.gyroAngle(), measurement.wheelPositions());

    // integrate to find difference in pose over time, add to pose estimate
    estimatedPose = poseEstimator.getEstimatedPosition();
  }

  public void addVisionMeasurement(VisionMeasurement measurement, Matrix<N3, N1> visionStdDevs) {
    poseEstimator.setVisionMeasurementStdDevs(visionStdDevs);
    poseEstimator.addVisionMeasurement(measurement.visionPose(), measurement.timestamp());
    estimatedPose = poseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose) {
    estimatedPose = pose;
    poseEstimator.resetPose(pose);
  }

  @AutoLogOutput(key = "Robot State/Estimated Pose")
  public Pose2d getEstimatedPose() {
    return estimatedPose;
  }

  @AutoLogOutput(key = "Robot State/Velocity")
  /* meters per second */
  public Translation2d getVelocity() {
    ChassisVelocities fieldSpeeds = robotSpeeds.toFieldRelative(estimatedPose.getRotation());
    return new Translation2d(fieldSpeeds.vx, fieldSpeeds.vy).rotateBy(Rotation2d.PI);
  }

  /* In inches because we are imperial... */
  @AutoLogOutput(key = "Robot State/Error")
  public double alignError() {
    return lastApproachPose.getTranslation().getDistance(estimatedPose.getTranslation())
        * 100
        / 2.54;
  }

  private Pose2d translateByVector(Pose2d pose, double mag, Rotation2d theta) {
    double scalarX = theta.getCos() * mag;
    double scalarY = theta.getSin() * mag;

    Transform2d transform = new Transform2d(scalarX, scalarY, Rotation2d.ZERO);
    return pose.transformBy(transform);
  }

  // translate + rotate
  private Pose2d offsetByVector(Pose2d pose, double mag, Rotation2d theta) {
    return translateByVector(pose, mag, theta).transformBy(new Transform2d(0, 0, theta));
  }

  /**
   * Gets the scuffed path planner built command for following a path to a certain pose
   *
   * @param approachPose2d
   * @return
   */
  public Command getPathPlannerApproachPoseCommand(Pose2d approachPose2d) {
    Logger.recordOutput("Robot State/Estimated Pose", estimatedPose);
    Logger.recordOutput("Robot State/Approach Pose", approachPose2d);

    Command finalPathfindingCommand = null;

    finalPathfindingCommand =
        AutoBuilder.pathfindToPose(approachPose2d, DriveConstants.ALIGN_PATH_CONSTRAINTS, 0.0);
    

    return finalPathfindingCommand;
  }

  public void addRobotSpeeds(ChassisVelocities chassisSpeeds) {
    this.robotSpeeds = chassisSpeeds;
  }

  public Pose2d getAlignPose() {
    return lastApproachPose;
  }

  @AutoLogOutput(key = "Robot State/isAllianceRed")
  public static boolean isAllianceRed() {
    // where true is red and false is blue
    var alliance = MatchState.getAlliance();
    if (RobotBase.isReal()) {
      return alliance.get() == Alliance.RED;
    }
    return true;
  }

}
