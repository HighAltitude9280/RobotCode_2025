package frc.robot.subsystems.extensor;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.resources.components.speedController.HighAltitudeMotorGroup;

public class Climber extends SubsystemBase {
  private HighAltitudeMotorGroup climberMotors;
  private Servo servo1;
  private Servo servo2;

  /** Creates a new Climber. */
  public Climber() {
    climberMotors = new HighAltitudeMotorGroup(RobotMap.CLIMBER_MOTOR_PORTS,
        RobotMap.CLIMBER_INVERTED_MOTORS_PORTS,
        RobotMap.CLIMBER_MOTOR_TYPES);

    // Inicializamos los servos con los puertos definidos en RobotMap
    if (servo1 == null) {
      servo1 = new Servo(RobotMap.CLIMBER_SERVO_1_PORT);
    }
    if (servo2 == null) {
      servo2 = new Servo(RobotMap.CLIMBER_SERVO_2_PORT);
    }
  }

  /** Mueve los servos a la posición deseada antes de activar el climber */
  public void prepareClimb() {
    if (servo1 != null) {
      servo1.set(1.0); // Mover a la posición 1
    }
    if (servo2 != null) {
      servo2.set(1.0); // Mover a la posición 1
    }
  }

  /** Mueve el Climber solo después de que los servos han cambiado de posición */
  public void driveClimber(double speed) {
    prepareClimb();
    climberMotors.setAll(speed);
  }

  /** Detiene el Climber */
  public void stopClimber() {
    climberMotors.setAll(0);
  }

  @Override
  public void periodic() {
    // Método llamado en cada ciclo de la scheduler
  }
}
