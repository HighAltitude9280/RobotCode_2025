// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.resources.components.speedController;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DriverStation;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

/** Add your docs here. */
public class HighAltitudeMotor {

    public enum TypeOfMotor {

        TALON_SRX, SPARK_MAX_BRUSHED

    }

    boolean inverted;
    private int port;
    private TypeOfMotor motorToUse;

    WPI_TalonSRX talonSRX; // CIM
    SparkMax sparkMax; // MOTOR BRUSHED

    SparkMaxConfig sparkMaxConfiguration = new SparkMaxConfig();

    public HighAltitudeMotor(int driveMotorPort, TypeOfMotor m) {
        this.port = driveMotorPort;
        motorToUse = m;

        switch (motorToUse) {

            case TALON_SRX:

                talonSRX = new WPI_TalonSRX(driveMotorPort);

                break;

            case SPARK_MAX_BRUSHED:

                sparkMax = new SparkMax(driveMotorPort, MotorType.kBrushed);

                break;
        }
    }

    public void set(double speed) {
        switch (motorToUse) {

            case TALON_SRX:

                talonSRX.set(ControlMode.PercentOutput, speed);

                break;

            case SPARK_MAX_BRUSHED:

                sparkMax.set(speed);

                break;

        }

    }

    public double getEncPosition() {
        switch (motorToUse) {

            case TALON_SRX:

                return talonSRX.getSelectedSensorPosition(0);

            case SPARK_MAX_BRUSHED:

                return sparkMax.getEncoder().getPosition();

            default:
                DriverStation.reportWarning("Encoder for " + motorToUse + " not found, returning 0!", true);
                return 0;
        }

    }

    public double getEncVelocity() {
        switch (motorToUse) {

            case TALON_SRX:

                return talonSRX.getSelectedSensorVelocity(0);

            case SPARK_MAX_BRUSHED:

                return sparkMax.getEncoder().getVelocity();

            default:
                DriverStation.reportWarning("Encoder for " + motorToUse + " not found, returning 0!", true);
                return 0;
        }
    }

    public void stopMotor() {
        switch (motorToUse) {

            case TALON_SRX:

                talonSRX.set(ControlMode.PercentOutput, 0);

                break;

            case SPARK_MAX_BRUSHED:

                sparkMax.set(0);

                break;
        }
    }

    public double getOutput() {
        switch (motorToUse) {

            case TALON_SRX:

                return talonSRX.getMotorOutputPercent();

            case SPARK_MAX_BRUSHED:

                return sparkMax.getOutputCurrent();

            default:
                DriverStation.reportWarning("Output for " + motorToUse + " not found, returning 0!", true);
                return 0;
        }
    }

    public int getPort() {
        return port;
    }

    public void setEncoderPosition(int value) {
        switch (motorToUse) {

            case TALON_SRX:

                talonSRX.setSelectedSensorPosition(value);

                break;

            case SPARK_MAX_BRUSHED:

                sparkMax.getEncoder().setPosition(value);

                break;
        }
    }

    public WPI_TalonSRX getTalonSRX() {
        return (WPI_TalonSRX) talonSRX;
    }

    public SparkMax getSparkMax() {
        return (SparkMax) sparkMax;
    }

    public void setBrakeMode(boolean doBrake) {
        switch (motorToUse) {

            case TALON_SRX:

                talonSRX.setNeutralMode(doBrake ? NeutralMode.Brake : NeutralMode.Coast);

                break;

            case SPARK_MAX_BRUSHED:

                sparkMaxConfiguration.idleMode(doBrake ? IdleMode.kBrake : IdleMode.kCoast);
                sparkMax.configure(sparkMaxConfiguration,
                        com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,
                        PersistMode.kPersistParameters);

                break;

            default:
                DriverStation.reportWarning("ERROR: The" + motorToUse + " not found", true);
                break;
        }
    }

    public void setInverted(boolean i) {
        inverted = i;

        switch (motorToUse) {

            case TALON_SRX:

                talonSRX.setInverted(i);

                break;

            case SPARK_MAX_BRUSHED:

                sparkMaxConfiguration.inverted(i);
                sparkMax.configure(sparkMaxConfiguration,
                        com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,
                        PersistMode.kPersistParameters);

                break;

            default:
                DriverStation.reportWarning("ERROR: The" + motorToUse + " not found", true);
                break;
        }

    }

    public boolean isInverted() {
        return inverted;
    }
/**
 * Gets the current percent output being applied to the motor.
 *
 * <p>Sign respects the controller’s inversion. This is not encoder velocity;
 * use your sensor API for RPM or m/s.
 *
 * @return value in [-1, 1], or 0 when no matching controller is active.
 * <p>Only supports Talon SRX and Spark MAX Brushed.
 */
    public double getSpeed() {
        switch (motorToUse) {

            case TALON_SRX:

                return talonSRX.get();

            case SPARK_MAX_BRUSHED:

                return sparkMax.getAppliedOutput();

            default:
                return 0;
        }
    }
/**
 * Reads the motor controller's bus (input) voltage.
 * <p>This is the battery voltage seen by the controller, not the motor's applied voltage
 * or percent output. 
 * Implementation note: the value is selected by {@code motorToUse}. 
 * <p>Only supports Talon SRX and Spark MAX Brushed.
 * @return bus voltage in volts; returns 0 if the controller type is unsupported or uninitialized.
 */
    public double getBusVoltage() {
        switch (motorToUse) {

            case TALON_SRX:

                return talonSRX.getBusVoltage();

            case SPARK_MAX_BRUSHED:

                return sparkMax.getBusVoltage();

            default:
                return 0;
        }
    }
/**
 * Sets the same voltage on every motor in this subsystem.
 * <p>Units are volts. Range is [-12, 12].
 * Negative values respect each motor's inversion setting.
 * This method does not clamp, ramp, or battery-compensate the value.
 * <p>Only supports Talon SRX and Spark MAX Brushed.
 * @param volts Voltage to apply to each motor (in volts).
 */
    public void setVoltage(double volts) {
        switch (motorToUse) {

            case TALON_SRX:

                talonSRX.set(ControlMode.Current, volts);

                break;

            case SPARK_MAX_BRUSHED:

                sparkMax.setVoltage(volts);

                break;
        }
    }
}