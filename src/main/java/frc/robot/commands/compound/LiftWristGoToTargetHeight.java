// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.compound;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.HighAltitudeConstants;
import frc.robot.HighAltitudeConstants.REEF_HEIGHT;
import frc.robot.commands.extensor.lift.control.LiftGoToTarget;
import frc.robot.commands.extensor.wrist.control.WristGoToTarget;
import frc.robot.Robot;
import frc.robot.subsystems.extensor.Lift;
import frc.robot.subsystems.extensor.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LiftWristGoToTargetHeight extends InstantCommand {
  Lift lift;
  Wrist wrist;

  REEF_HEIGHT height;
  boolean goingUp;

  double liftTarget;
  double wristTarget;

  boolean isFinished = false;
  boolean secondPart = false;

  public LiftWristGoToTargetHeight(REEF_HEIGHT height) {
    wrist = Robot.getRobotContainer().getWrist();
    lift = Robot.getRobotContainer().getLift();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(wrist, lift);
    this.height = height;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if (Robot.isCoralMode()) {
      liftTarget = HighAltitudeConstants.LIFT_CORAL_POSITIONS[height.getID()];
      wristTarget = HighAltitudeConstants.WRIST_CORAL_POSITIONS[height.getID()];

    } else {
      liftTarget = HighAltitudeConstants.LIFT_ALGAE_POSITIONS[height.getID()];
      wristTarget = HighAltitudeConstants.WRIST_ALGAE_POSITIONS[height.getID()];
    }
    goingUp = liftTarget > Robot.getRobotContainer().getLift().getLiftPosMeters();

    if (goingUp)
      (new SequentialCommandGroup(
          new LiftGoToTarget(HighAltitudeConstants.LIFT_MAX_POWER, liftTarget,
              HighAltitudeConstants.LIFT_ARRIVE_OFFSET),
          new WristGoToTarget(wristTarget, HighAltitudeConstants.WRIST_DRIVE_SPEED))).schedule();
    else
      (new SequentialCommandGroup(new WristGoToTarget(wristTarget, HighAltitudeConstants.WRIST_DRIVE_SPEED),
          new LiftGoToTarget(HighAltitudeConstants.LIFT_MAX_POWER, liftTarget,
              HighAltitudeConstants.LIFT_ARRIVE_OFFSET)))
          .schedule();
  }
}
