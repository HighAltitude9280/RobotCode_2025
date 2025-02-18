package frc.robot.subsystems;

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

  public CANdleSubsystem() {
    candle = new CANdle(0); // ID del CANdle en el bus CAN

    // Configuración de LEDs
    CANdleConfiguration config = new CANdleConfiguration();
    config.statusLedOffWhenActive = true; // Apaga el LED de estado cuando está activo
    config.stripType = CANdle.LEDStripType.RGB; // Tipo de LED (RGB o RGBW)
    config.brightnessScalar = 0.5; // Brillo (0.0 a 1.0)

    candle.configAllSettings(config);

    // Inicializar animación arcoíris
    rainbowAnimation = new RainbowAnimation(1.0, 0.5, 64); // Velocidad, Brillo, Número de LEDs

    // Crear animación de fuego
    fireAnimation = new FireAnimation();
    // 🔥 Velocidad, Intensidad, LEDs, Sparking, Reverse Direction
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
}