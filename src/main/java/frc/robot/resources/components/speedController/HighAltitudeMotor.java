// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.resources.components.speedController;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DriverStation;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

/** Add your docs here. */
public class HighAltitudeMotor {

    public enum TypeOfMotor {

        TALON_SRX, TALON_FX,
        SPARK_MAX_BRUSHLESS, SPARK_MAX_BRUSHED,
        SPARK_FLEX

    }

    boolean inverted;
    private int port;
    private TypeOfMotor motorToUse;

    WPI_TalonSRX talonSRX;
    TalonFX talonFX; // KRAKEN X60 , FALCON 500
    SparkMax sparkMax; // NEO, NEO 550, MOTOR BRUSHED
    SparkFlex sparkFlex; // NEO VORTEX

    TalonFXConfiguration talonFXConfiguration;

    SparkMaxConfig sparkMaxConfiguration = new SparkMaxConfig();
    SparkFlexConfig sparkFlexConfiguration = new SparkFlexConfig();

    public HighAltitudeMotor(int driveMotorPort, TypeOfMotor m) {
        this.port = driveMotorPort;
        motorToUse = m;

        switch (motorToUse) {

            case TALON_SRX:

                talonSRX = new WPI_TalonSRX(driveMotorPort);

                break;

            case TALON_FX:

                talonFXConfiguration = new TalonFXConfiguration();
                talonFX = new TalonFX(driveMotorPort);
                talonFX.getConfigurator().apply(talonFXConfiguration);
                break;

            case SPARK_MAX_BRUSHED:

                sparkMax = new SparkMax(driveMotorPort, MotorType.kBrushed);

                break;

            case SPARK_MAX_BRUSHLESS:
                sparkMaxConfiguration = new SparkMaxConfig();
                sparkMax = new SparkMax(driveMotorPort, MotorType.kBrushless);
                sparkMaxConfiguration.inverted(false).idleMode(IdleMode.kCoast);
                sparkMaxConfiguration.encoder.positionConversionFactor(1).velocityConversionFactor(1);
                sparkMaxConfiguration.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0, 0,
                        0);
                sparkMax.configure(sparkMaxConfiguration,
                        com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

                break;

            case SPARK_FLEX:

                sparkFlex = new SparkFlex(driveMotorPort, MotorType.kBrushless);
                sparkFlexConfiguration.inverted(false).idleMode(IdleMode.kCoast);
                sparkFlexConfiguration.encoder.positionConversionFactor(1).velocityConversionFactor(1);
                sparkFlexConfiguration.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0, 0, 0);
                sparkFlex.configure(sparkFlexConfiguration,
                        com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

                break;
        }
    }

    public void setMotorPID(double kP, double kI, double kD) {
        if (motorToUse == null) {
            DriverStation.reportWarning("Error: The motor " + motorToUse + " is not found, check your constructor!",
                    true);
            return;
        }

        switch (motorToUse) {

            case TALON_FX:
                if (talonFX == null) {
                    DriverStation.reportWarning(
                            "Error: The motor " + motorToUse + " is not found, check your constructor!", true);
                    return;
                }

                break;

            case SPARK_MAX_BRUSHLESS:
                if (sparkMax == null) {
                    DriverStation.reportWarning(
                            "Error: The motor " + motorToUse + " is not found, check your constructor!", true);
                    return;
                }

                sparkMaxConfiguration.closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .pid(kP, kI, kD);

                sparkMax.configure(sparkMaxConfiguration,
                        com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,
                        PersistMode.kPersistParameters);

                break;

            case SPARK_FLEX:
                if (sparkFlex == null) {
                    DriverStation.reportWarning(
                            "Error: The motor " + motorToUse + " is not found, check your constructor!", true);
                    return;
                }

                sparkFlexConfiguration.closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .pid(kP, kI, kD);

                sparkFlex.configure(sparkFlexConfiguration,
                        com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,
                        PersistMode.kPersistParameters);

                break;

            default:
                System.err.println("Error: Configuración PID no soportada para este tipo de motor.");
                DriverStation.reportWarning("Error: PID Config is not supported for this type of motor", true);

        }
    }

    public void set(double speed) {
        switch (motorToUse) {

            case TALON_SRX:

                talonSRX.set(ControlMode.PercentOutput, speed);

                break;

            case TALON_FX:

                talonFX.set(speed);

                break;

            case SPARK_MAX_BRUSHED:

                sparkMax.set(speed);

                break;

            case SPARK_MAX_BRUSHLESS:

                sparkMax.set(speed);

                break;

            case SPARK_FLEX:

                sparkFlex.set(speed);

                break;
        }

    }

    public double getEncPosition() {
        switch (motorToUse) {

            case TALON_SRX:

                return talonSRX.getSelectedSensorPosition(0);

            case TALON_FX:

                return talonFX.getPosition().getValueAsDouble();

            case SPARK_MAX_BRUSHED:

                return sparkMax.getEncoder().getPosition();

            case SPARK_MAX_BRUSHLESS:

                return sparkMax.getEncoder().getPosition();

            case SPARK_FLEX:

                return sparkFlex.getEncoder().getPosition();

            default:
                DriverStation.reportWarning("Encoder for " + motorToUse + " not found, returning 0!", true);
                return 0;
        }

    }

    public double getEncVelocity() {
        switch (motorToUse) {

            case TALON_SRX:

                return talonSRX.getSelectedSensorVelocity(0);

            case TALON_FX:

                return talonFX.getVelocity().getValueAsDouble();

            case SPARK_MAX_BRUSHED:

                return sparkMax.getEncoder().getVelocity();

            case SPARK_MAX_BRUSHLESS:

                return sparkMax.getEncoder().getVelocity();

            case SPARK_FLEX:

                return sparkFlex.getEncoder().getVelocity();

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

            case TALON_FX:

                talonFX.set(0);

                break;

            case SPARK_MAX_BRUSHED:

                sparkMax.set(0);

                break;

            case SPARK_MAX_BRUSHLESS:

                sparkMax.set(0);

                break;

            case SPARK_FLEX:

                sparkFlex.set(0);

                break;
        }
    }

    public double getOutput() {
        switch (motorToUse) {

            case TALON_SRX:

                return talonSRX.getMotorOutputPercent();

            case TALON_FX:

                return talonFX.getMotorVoltage().getValueAsDouble();

            case SPARK_MAX_BRUSHED:

                return sparkMax.getOutputCurrent();

            case SPARK_MAX_BRUSHLESS:

                return sparkMax.getOutputCurrent();

            case SPARK_FLEX:

                return sparkFlex.getOutputCurrent();

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

            case TALON_FX:

                talonFX.setPosition(value);

                break;

            case SPARK_MAX_BRUSHED:

                sparkMax.getEncoder().setPosition(value);

                break;

            case SPARK_MAX_BRUSHLESS:

                sparkMax.getEncoder().setPosition(value);

                break;

            case SPARK_FLEX:

                sparkFlex.getEncoder().setPosition(value); // getEncoder o getAbsoluteEncoder

                break;
        }
    }

    public WPI_TalonSRX getTalonSRX() {
        return (WPI_TalonSRX) talonSRX;
    }

    public TalonFX getTalonFX() {
        return (TalonFX) talonFX;
    }

    public SparkMax getSparkMax() {
        return (SparkMax) sparkMax;
    }

    public SparkFlex getSparkFlex() {
        return (SparkFlex) sparkFlex;
    }

    public void setBrakeMode(boolean doBrake) {
        switch (motorToUse) {

            case TALON_SRX:

                talonSRX.setNeutralMode(doBrake ? NeutralMode.Brake : NeutralMode.Coast);

                break;

            case TALON_FX:

                talonFX.setNeutralMode(doBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast);

                break;

            case SPARK_MAX_BRUSHED:

                sparkMaxConfiguration.idleMode(doBrake ? IdleMode.kBrake : IdleMode.kCoast);
                sparkMax.configure(sparkMaxConfiguration,
                        com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,
                        PersistMode.kPersistParameters);

                break;

            case SPARK_MAX_BRUSHLESS:

                sparkMaxConfiguration.idleMode(doBrake ? IdleMode.kBrake : IdleMode.kCoast);
                sparkMax.configure(sparkMaxConfiguration,
                        com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,
                        PersistMode.kPersistParameters);

                break;

            case SPARK_FLEX:

                sparkFlexConfiguration.idleMode(doBrake ? IdleMode.kBrake : IdleMode.kCoast);
                sparkFlex.configure(sparkFlexConfiguration,
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

            case TALON_FX:
                talonFX.setInverted(i);

                break;

            case SPARK_MAX_BRUSHED:

                sparkMaxConfiguration.inverted(i);
                sparkMax.configure(sparkMaxConfiguration,
                        com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,
                        PersistMode.kPersistParameters);

                break;

            case SPARK_MAX_BRUSHLESS:

                sparkMaxConfiguration.inverted(i);
                sparkMax.configure(sparkMaxConfiguration,
                        com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,
                        PersistMode.kPersistParameters);

                break;

            case SPARK_FLEX:

                sparkFlexConfiguration.inverted(i);
                sparkFlex.configure(sparkFlexConfiguration,
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

    public double get()
    {
        switch (motorToUse) {

            case TALON_SRX:

                return talonSRX.get();

            case TALON_FX:

                return talonFX.get();

            case SPARK_MAX_BRUSHED:

                return sparkMax.get();

            case SPARK_MAX_BRUSHLESS:

                return sparkMax.get();

            case SPARK_FLEX:

                return sparkFlex.get();
            default:
                return 0;
        }
    }

    public void setVoltage(double volts) {
        switch (motorToUse) {

            case TALON_SRX:

                talonSRX.set(ControlMode.Current, volts);

                break;

            case TALON_FX:

                talonFX.setVoltage(volts);

                break;

            case SPARK_MAX_BRUSHED:

                sparkMax.setVoltage(volts);

                break;

            case SPARK_MAX_BRUSHLESS:

                sparkMax.setVoltage(volts);

                break;

            case SPARK_FLEX:

                sparkFlex.setVoltage(volts);

                break;
        }
    }
}