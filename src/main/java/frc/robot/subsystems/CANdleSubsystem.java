package frc.robot.subsystems;

import java.util.Random;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.RainbowAnimation;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CANdleSubsystem extends SubsystemBase {
  private final CANdle candle;
  private Animation rainbowAnimation;
  private Animation fireAnimation;

  private final Random random;
  private final int ledCount = 24; // Ajusta según la cantidad de LEDs
  private final int baseR = 0; // Rojo (0)
  private final int baseG = 146; // Verde (146)
  private final int baseB = 128; // Azul (128)

  public CANdleSubsystem() {
    candle = new CANdle(0); // ID del CANdle en el bus CAN

    // Configuración de LEDs
    CANdleConfiguration config = new CANdleConfiguration();
    config.statusLedOffWhenActive = true; // Apaga el LED de estado cuando está activo
    config.stripType = CANdle.LEDStripType.RGB; // Tipo de LED (RGB o RGBW)
    config.brightnessScalar = 0.1; // Brillo (0.0 a 1.0)

    candle.configAllSettings(config);

    // Inicializar animación arcoíris
    rainbowAnimation = new RainbowAnimation(0.1, 0.5, 22); // Velocidad, Brillo, Número de LEDs

    // Crear animación de fuego
    fireAnimation = new FireAnimation(0.1, 0.25, 120, 1, 1);
    // 🔥 Velocidad, Intensidad, LEDs, Sparking, Reverse Direction
    random = new Random();
  }

  // Método para cambiar el color de los LEDs
  public void setLEDColor(int r, int g, int b) {
    candle.setLEDs(r, g, b);
  }

  // Activar la animación arcoíris
  public void startRainbowAnimation() {
    candle.animate(rainbowAnimation);
  }

  // Método para activar la animación de fuego verde
  public void startFireAnimation() {

    candle.animate(fireAnimation);
  }

  public void flameMode() {
    for (int i = 0; i < ledCount; i++) {
      int flicker = random.nextInt(50) - 25; // Oscila entre -25 y +25

      int g = clamp(baseG + flicker, 0, 255);
      int b = clamp(baseB + flicker / 2, 0, 255);

      // ✅ Corrige el uso de setLEDs() para LEDs individuales
      candle.setLEDs(baseR, g, b, 0, i, 1);
    }
  }

  // 🔥 Función clamp agregada para evitar errores
  private int clamp(int value, int min, int max) {
    return Math.max(min, Math.min(max, value));
  }
}