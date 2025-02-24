// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.extensor.gripper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.HighAltitudeConstants;
import frc.robot.Robot;
import frc.robot.subsystems.manipulator.Gripper;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeUntilCoral extends Command {
  Gripper gripper;
  double speed;

  /** Creates a new IntakeUntilCoral. */
  public IntakeUntilCoral() {
    gripper = Robot.getRobotContainer().getGripper();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(gripper);
    speed = HighAltitudeConstants.GRIPPER_INTAKE_SPEED;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    gripper.driveGripper(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    gripper.stopGripper();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return gripper.getColorSensorInput();
  }
}
