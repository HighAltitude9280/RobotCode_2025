// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.HighAltitudeConstants.REEF_SIDE;
import frc.robot.commands.leds.SetLEDColor;
import frc.robot.commands.modes.SetCoralMode;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  public static RobotContainer robotContainer;
  private Command m_autonomousCommand;

  // Branch selection modes.
  private static boolean leftMode = true;

  public static void setLeftMode(boolean leftMode) {
    Robot.leftMode = leftMode;
  }

  public static boolean isLeftMode() {
    return leftMode;
  }

  private static boolean frontMode = true;

  public static void setFrontMode(boolean frontMode) {
    Robot.frontMode = frontMode;
  }

  public static boolean isFrontMode() {
    return frontMode;
  }

  private static boolean coralMode = true;

  public static void setCoralMode(boolean coralMode) {
    Robot.coralMode = coralMode;
  }

  public static boolean isCoralMode() {
    return coralMode;
  }

  private static REEF_SIDE reef_mode = REEF_SIDE.CENTER;

  public static REEF_SIDE getReefMode() {
    return reef_mode;
  }

  public static void setReefMode(REEF_SIDE mode) {
    reef_mode = mode;
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  public Robot() {
    robotContainer = new RobotContainer();
    getRobotContainer().ConfigureButtonBindings();
    getRobotContainer().generateAutos();

  }

  @Override
  public void robotInit() {
    setCoralMode(coralMode);
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
  
    putSmartDashboardMatchMode();

    if(getRobotContainer().getLift().getTopLimitSwitch())
      getRobotContainer().toggleBrakeModeAllMotors();

  }

  void putSmartDashboardMatchMode()
  {
    SmartDashboard.putBoolean("Left mode", leftMode);
    SmartDashboard.putBoolean("Coral mode", coralMode);
    SmartDashboard.putString("Reef mode", reef_mode.name());
    SmartDashboard.putBoolean("Field oriented mode", getRobotContainer().getSwerveDriveTrain().getIsFieldOriented());
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
    // getRobotContainer().getSwerveDriveTrain().setModulesBrakeMode(true);
    m_autonomousCommand = robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    // getRobotContainer().getSwerveDriveTrain().setModulesBrakeMode(true);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    // getRobotContainer().getCaNdleSubsystem().startFireAnimation(); TODO: Barrera
    // chamba
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    getRobotContainer().putAutoChooser();
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

  public static RobotContainer getRobotContainer() {
    return robotContainer;
  }
}
