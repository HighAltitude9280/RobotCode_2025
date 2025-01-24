// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.resources.components.speedController.HighAltitudeMotor;
import frc.robot.resources.components.speedController.HighAltitudeMotor.TypeOfMotor;
import frc.robot.resources.joysticks.HighAltitudeJoystick;
import frc.robot.resources.joysticks.HighAltitudeJoystick.AxisType;
import frc.robot.resources.joysticks.HighAltitudeJoystick.ButtonType;
import frc.robot.resources.joysticks.HighAltitudeJoystick.JoystickType;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  HighAltitudeMotor sparkMaxTest = new HighAltitudeMotor(39, TypeOfMotor.SPARK_MAX_BRUSHLESS);
  HighAltitudeMotor talonFXTest = new HighAltitudeMotor(12,
      TypeOfMotor.TALON_FX);

  // Crea el objeto Orchestra
  private HighAltitudeJoystick pilot = new HighAltitudeJoystick(0, JoystickType.XBOX);

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  public Robot() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    pilot.onTrue(ButtonType.A, new InstantCommand(() -> sparkMaxTest.setBrakeMode(true)));
    pilot.onTrue(ButtonType.X, new InstantCommand(() -> sparkMaxTest.setBrakeMode(false)));

    pilot.onTrue(ButtonType.B, new InstantCommand(() -> sparkMaxTest.setInverted(true)));
    pilot.onTrue(ButtonType.Y, new InstantCommand(() -> sparkMaxTest.setInverted(false)));

    pilot.onTrue(ButtonType.POV_N, new InstantCommand(() -> talonFXTest.setBrakeMode(true)));
    pilot.onTrue(ButtonType.POV_S, new InstantCommand(() -> talonFXTest.setBrakeMode(false)));

    pilot.onTrue(ButtonType.POV_W, new InstantCommand(() -> talonFXTest.setInverted(true)));
    pilot.onTrue(ButtonType.POV_E, new InstantCommand(() -> talonFXTest.setInverted(false)));

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("SparkMAX Encoder Position Value", sparkMaxTest.getEncPosition());
    SmartDashboard.putNumber("SparkMAX Encoder Velocity Value", sparkMaxTest.getEncVelocity());

    SmartDashboard.putNumber("TalonFX Encoder Position Value", talonFXTest.getEncPosition());
    SmartDashboard.putNumber("TalonFX Encoder Velocity Value", talonFXTest.getEncVelocity());

  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the SendableChooser make sure to add
   * them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double input = pilot.getAxis(AxisType.LEFT_Y);
    sparkMaxTest.set(input * 0.5);

    double input2 = pilot.getAxis(AxisType.RIGHT_Y);
    talonFXTest.set(input2 * 0.2);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
