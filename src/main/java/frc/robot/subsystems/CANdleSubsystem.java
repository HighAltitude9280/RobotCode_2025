package frc.robot.subsystems;

import java.util.Random;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CANdleSubsystem extends SubsystemBase {
  private final CANdle candle;
  private Animation rainbowAnimation;
  private Animation fireAnimation;
  private Animation flareAnimation;

  private final Random random;
  private final int ledCount = 34; // Adjust based on LED count
  private final int baseR = 0; // Red (0)
  private final int baseG = 146; // Green (146)
  private final int baseB = 128; // Blue (128)

  int brightness = 0;
  boolean increasing = true;

  public CANdleSubsystem() {
    candle = new CANdle(0); // CANdle ID on the CAN bus

    // LED Configuration
    CANdleConfiguration config = new CANdleConfiguration();
    config.statusLedOffWhenActive = true; // Turn off status LED when active
    config.stripType = CANdle.LEDStripType.RGB; // LED strip type (RGB or RGBW)
    config.brightnessScalar = 0.1; // Brightness (0.0 to 1.0)

    candle.configAllSettings(config);

    // Initialize rainbow animation
    rainbowAnimation = new RainbowAnimation(0.1, 0.5, ledCount); // Speed, Brightness, LED Count

    // Initialize fire animation
    fireAnimation = new FireAnimation(0.3, 0.3, ledCount, 0.5, 0.2);
    // Speed, Intensity, LEDs, Sparking, Reverse Direction

    flareAnimation = new StrobeAnimation(0, 255, 0, 0, 0.1, ledCount);
    // Red, Green, Blue, White, Speed, LED Count, Twinkle Percent

    random = new Random();
  }

  // Set static LED color
  public void setLEDColor(int r, int g, int b) {
    candle.setLEDs(r, g, b);
  }

  // Activate rainbow animation
  public void startRainbowAnimation() {
    candle.animate(rainbowAnimation);
  }

  // Activate green fire animation
  public void startFireAnimation() {
    candle.animate(fireAnimation);
  }

  // Activate flare animation
  public void startFlareAnimation() {
    candle.animate(flareAnimation);
  }

  public void setGreenFireAnimation() {
    for (int i = 0; i < ledCount; i++) {
      int green = 100 + random.nextInt(156); // Valor entre 100 y 255
      int blue = random.nextInt(100); // Valor entre 0 y 99
      candle.setLEDs(0, green, blue, 0, i, 1);
    }
  }

  // Flickering flame mode (manual)
  public void flameMode() {
    for (int i = 0; i < ledCount; i++) {
      int flicker = random.nextInt(50) - 25; // Oscillates between -25 and +25

      int g = clamp(baseG + flicker, 0, 255);
      int b = clamp(baseB + flicker / 2, 0, 255);

      // Use setLEDs() correctly for individual LEDs
      candle.setLEDs(baseR, g, b, 0, i, 1);
    }
  }

  // Turn off LEDs
  public void turnOff() {
    candle.setLEDs(0, 0, 0);
    candle.animate(null, 0);
  }

  // Clamp function to prevent out-of-bounds values
  private int clamp(int value, int min, int max) {
    return Math.max(min, Math.min(max, value));
  }
}