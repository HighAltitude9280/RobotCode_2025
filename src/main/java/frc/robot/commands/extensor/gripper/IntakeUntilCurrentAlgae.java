// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.extensor.gripper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.manipulator.Gripper;

public class IntakeUntilCurrentAlgae extends Command {
  private final Gripper gripper;

  /** Creates a new IntakeUntilCurrentCoral. */
  public IntakeUntilCurrentAlgae() {
    gripper = Robot.getRobotContainer().getGripper();
    addRequirements(gripper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    gripper.gripperAlgaeCurrent();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Asegurar que el gripper sigue encendido hasta que la corriente supere el
    // umbral

    gripper.gripperAlgaeCurrent();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    gripper.stopGripper();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return gripper.isCurrentThresholdExceededAlgae();
  }
}
