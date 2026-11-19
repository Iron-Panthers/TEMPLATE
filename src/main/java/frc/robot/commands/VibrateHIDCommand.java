package frc.robot.commands;

import org.wpilib.driverstation.GenericHID;
import org.wpilib.driverstation.GenericHID.RumbleType;
import org.wpilib.system.Timer;
import org.wpilib.command2.Command;

public class VibrateHIDCommand extends Command {
  private GenericHID hid;
  private double duration;
  private double startTime;
  private double strength;

  /**
   * makes a command to vibrate a controller
   *
   * @param controller the controller to call the rumble methods on
   * @param duration the time (seconds) to vibrate for
   * @param strength the strength [0, 1] double of the vibration
   */
  public VibrateHIDCommand(GenericHID hid, double duration, double strength) {
    this.hid = hid;
    this.duration = duration;
    this.strength = strength;
    startTime = 0;
  }

  @Override
  public void initialize() {
    startTime = Timer.getTimestamp();
    hid.setRumble(RumbleType.RIGHT_RUMBLE, strength);
    hid.setRumble(RumbleType.LEFT_RUMBLE, strength);
  }

  @Override
  public void end(boolean interrupted) {
    hid.setRumble(RumbleType.RIGHT_RUMBLE, 0);
    hid.setRumble(RumbleType.LEFT_RUMBLE, 0);
  }

  @Override
  public boolean isFinished() {
    return Timer.getTimestamp() >= startTime + duration;
  }
}
