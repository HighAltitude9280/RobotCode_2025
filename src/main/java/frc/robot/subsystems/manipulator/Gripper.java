// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manipulator;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HighAltitudeConstants;
import frc.robot.RobotMap;
import frc.robot.resources.components.speedController.HighAltitudeMotorGroup;

public class Gripper extends SubsystemBase {
  HighAltitudeMotorGroup gripperMotors;

  DigitalInput gripperDigitalInput;

  // Sensor de color
  ColorSensorV3 colorSensor;
  ColorMatch colorMatcher;

  private final double CURRENT_THRESOLD = -2.3;
  boolean coralInGripper;
  // Definir el color "blanco del coral"
  private final Color coralWhite = new Color(0.30, 0.45, 0.25); // Ajusta estos valores
  private final double tolerance = 0.015; // Ajusta según pruebas

  /** Creates a new Gripper. */
  public Gripper() {
    gripperMotors = new HighAltitudeMotorGroup(RobotMap.GRIPPER_MOTOR_PORTS, RobotMap.GRIPPER_INVERTED_MOTORS_PORTS,
        RobotMap.GRIPPER_MOTOR_TYPES);

    gripperMotors.setEncoderInverted(RobotMap.GRIPPER_ENCODER_IS_INVERTED);

    if (RobotMap.GRIPPER_DIGITAL_INPUT_IS_AVAILABLE)
      gripperDigitalInput = new DigitalInput(RobotMap.GRIPPER_DIGITAL_INPUT_PORT);
    if (RobotMap.GRIPPER_I2C_PORT_IS_AVAILABLE) {
      colorSensor = new ColorSensorV3(I2C.Port.kOnboard); // Cambiar si está en MXP
      colorMatcher = new ColorMatch();
      colorMatcher.addColorMatch(coralWhite);
    }
  }

  public void driveGripper(double speed) {
    gripperMotors.setAll(speed);
  }

  public void gripperIn() {
    gripperMotors.setAll(HighAltitudeConstants.GRIPPER_IN_SPEED);
  }

  /** Verifica si la corriente del motor ha superado el umbral */
  public boolean isCurrentThresholdExceeded() {
    double current = gripperMotors.getMotors().get(0).getOutput();
    System.out.println("Current:" + current);
    return current > CURRENT_THRESOLD;
  }

  public void gripperInCurrent() {
    if (!isCurrentThresholdExceeded())
      gripperMotors.setAll(HighAltitudeConstants.GRIPPER_INTAKE_SPEED);
    else {
      stopGripper();
    }
  }

  public void gripperOut(double speed) {
    gripperMotors.setAll(speed);
  }

  public void stopGripper() {
    gripperMotors.setAll(0);
  }

  public boolean getDigitalInput() {
    if (RobotMap.GRIPPER_DIGITAL_INPUT_IS_AVAILABLE)
      return gripperDigitalInput.get();
    return false;
  }

  /** Getter para detectar si el color blanco del coral está presente */
  public boolean getColorSensorInput() {
    if (RobotMap.GRIPPER_I2C_PORT_IS_AVAILABLE) {
      Color detectedColor = colorSensor.getColor();

      // Comparación con el blanco del coral
      boolean isCoralWhite = Math.abs(detectedColor.red - coralWhite.red) < tolerance &&
          Math.abs(detectedColor.green - coralWhite.green) < tolerance &&
          Math.abs(detectedColor.blue - coralWhite.blue) < tolerance;

      coralInGripper = isCoralWhite;
      return coralInGripper;
    }
    return false; // Retorna false si el sensor no está disponible
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
