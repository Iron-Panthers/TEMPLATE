package frc.robot;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.kinematics.ChassisVelocities;
import frc.robot.Constants.RobotType;
import frc.robot.subsystems.swerve.DriveConstants;
import frc.robot.utility.BlankSimulatedArena;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

public class RobotSimState {

  private RobotSimState() {
    // init the arena (drive sim only, no game piece placement)
    BlankSimulatedArena arena = new BlankSimulatedArena();
    // arena.setEfficiencyMode(true);
    arena.clearGamePieces();

    // Add the drive simulation
    driveSimulation =
        new SwerveDriveSimulation(
            DriveConstants.mapleSimConfig, RobotState.getInstance().getEstimatedPose());
    arena.addDriveTrainSimulation(driveSimulation);

    SimulatedArena.overrideInstance(arena);
  }

  // Singleton instance
  private static RobotSimState instance = null;

  public static RobotSimState getInstance() {
    if (Constants.getRobotType() != RobotType.SIM) {
      // idiot proofing
      System.out.println(
          "WARNING: YOU ARE TRYING TO ACCESS ROBOT SIM STATE FROM AN ACTUAL ROBOT -- THIS IS A CODE"
              + " ERROR");
    }
    if (instance == null) instance = new RobotSimState();
    return instance;
  }

  // Drive simulation
  private SwerveDriveSimulation driveSimulation;

  public SwerveDriveSimulation getDriveSimulation() {
    return driveSimulation;
  }

  // Get attributes of physical drivebase
  public Pose2d getRobotPose2d() {
    return driveSimulation.getSimulatedDriveTrainPose();
  }

  public Pose3d getRobotPose3d() {
    Pose2d robotPose2d = driveSimulation.getSimulatedDriveTrainPose();
    return new Pose3d(
        new Translation3d(robotPose2d.getX(), robotPose2d.getY(), 0.0),
        new Rotation3d(0, 0, robotPose2d.getRotation().getRadians()));
  }

  public ChassisVelocities getChassisVelocitiesFieldRelative() {
    return driveSimulation.getDriveTrainSimulatedChassisVelocitiesFieldRelative();
  }
}
