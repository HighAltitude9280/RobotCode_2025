// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.extensor.gripper;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.HighAltitudeConstants;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.manipulator.Gripper;

public class IntakeUntilCurrentCoral extends Command {
  private final Gripper gripper;
  private final double speed;

  /** Creates a new IntakeUntilCurrentCoral. */
  public IntakeUntilCurrentCoral() {
    gripper = Robot.getRobotContainer().getGripper();
    addRequirements(gripper);
    speed = HighAltitudeConstants.GRIPPER_INTAKE_SPEED;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    gripper.gripperInCurrent();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Asegurar que el gripper sigue encendido hasta que la corriente supere el
    // umbral
    if (!gripper.isCurrentThresholdExceeded()) {
      gripper.gripperInCurrent();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    gripper.stopGripper();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return gripper.isCurrentThresholdExceeded();
  }
}
