package frc.robot.subsystems.rgb;

import org.wpilib.hardware.led.AddressableLED;
import org.wpilib.hardware.led.AddressableLEDBuffer;
import org.wpilib.hardware.led.LEDPattern;

public class RGBIOAddressableLED implements RGBIO {
  private final AddressableLED addressableLED;
  private final AddressableLEDBuffer addressableLEDBuffer;
  private LEDPattern pattern;

  public RGBIOAddressableLED() {
    addressableLED = new AddressableLED(RGBConstants.RGB_CONFIGS.id());
    addressableLED.setLength(RGBConstants.RGB_CONFIGS.numLEDs());
    addressableLEDBuffer = new AddressableLEDBuffer(RGBConstants.RGB_CONFIGS.numLEDs());
    addressableLED.setData(addressableLEDBuffer);
    addressableLED.start();
    // addressableLED.configLOSBehavior(true);
    // addressableLED.configLEDType(LEDStripType.GRB);
  }

  public void updateInputs(RGBIOInputs inputs) {}

  public void displayMessage(RGBConstants.RGBMessage message) {
    message.getPattern().applyTo(addressableLEDBuffer);
    addressableLED.setData(addressableLEDBuffer);
  }

  public void clear() {}
}
