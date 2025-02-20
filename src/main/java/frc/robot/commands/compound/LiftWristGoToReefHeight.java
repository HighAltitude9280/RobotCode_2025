// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.compound;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.HighAltitudeConstants;
import frc.robot.Robot;
import frc.robot.subsystems.extensor.Lift;
import frc.robot.subsystems.extensor.Wrist;
import frc.robot.subsystems.manipulator.Gripper;
import frc.robot.HighAltitudeConstants.REEF_HEIGHT;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LiftWristGoToReefHeight extends Command {

  Lift lift;
  Wrist wrist;

  REEF_HEIGHT height;
  boolean goingUp;

  double liftTarget;
  double wristTarget;

  boolean isFinished = false;

  public LiftWristGoToReefHeight(REEF_HEIGHT height) {
    this.height = height;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wrist = Robot.getRobotContainer().getWrist();
    lift = Robot.getRobotContainer().getLift();
    addRequirements(wrist, lift);

    if (Robot.isCoralMode()) {
      liftTarget = HighAltitudeConstants.LIFT_CORAL_POSITIONS[height.getID()];
      wristTarget = HighAltitudeConstants.WRIST_CORAL_POSITIONS[height.getID()];

    } else {
      liftTarget = HighAltitudeConstants.LIFT_ALGAE_POSITIONS[height.getID()];
      wristTarget = HighAltitudeConstants.WRIST_ALGAE_POSITIONS[height.getID()];
    }
    goingUp = liftTarget > Robot.getRobotContainer().getLift().getLiftPosMeters();

    if (goingUp)
      Robot.getRobotContainer().getLift().setTarget(liftTarget);
    else
      Robot.getRobotContainer().getWrist().setTarget(wristTarget);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (goingUp && Robot.getRobotContainer().getLift().onTarget()) {
      Robot.getRobotContainer().getWrist().setTarget(wristTarget);
      // return;
    } else if (Robot.getRobotContainer().getWrist().onTarget()) {
      Robot.getRobotContainer().getLift().setTarget(liftTarget);
      // return;
    }
    lift.controlPosition(4);
    wrist.mantainTarget(HighAltitudeConstants.WRIST_DRIVE_SPEED);
    SmartDashboard.putBoolean("wGoingUp", goingUp);
    isFinished = Robot.getRobotContainer().getLift().onTarget() && Robot.getRobotContainer().getWrist().onTarget();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
