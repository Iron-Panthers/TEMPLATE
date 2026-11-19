package frc.robot.subsystems.swerve;

import org.wpilib.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
  @AutoLog
  class GyroIOInputs {
    public boolean isConnected = false;
    public Rotation2d yawPosition = new Rotation2d();
    public double yawVelocityRadPerSec = 0;
  }

  default void updateInputs(GyroIOInputs inputs) {}
}
