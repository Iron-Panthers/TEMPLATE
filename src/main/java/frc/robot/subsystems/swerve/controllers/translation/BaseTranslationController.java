package frc.robot.subsystems.swerve.controllers.translation;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import java.util.function.Supplier;

public abstract class BaseTranslationController {
  protected final Supplier<Rotation2d> yawSupplier;

  /* teleop control with specified yaw supplier, typically "arbitrary" yaw */
  public BaseTranslationController(Supplier<Rotation2d> yawSupplier) {
    this.yawSupplier = yawSupplier;
  }

  public abstract ChassisVelocities update();
}
