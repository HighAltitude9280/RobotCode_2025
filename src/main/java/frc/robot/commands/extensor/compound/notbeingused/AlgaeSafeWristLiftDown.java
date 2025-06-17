// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.extensor.compound.notbeingused;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.HighAltitudeConstants;
import frc.robot.Robot;
import frc.robot.commands.extensor.lift.control.LiftGoToTarget;
import frc.robot.commands.extensor.wrist.control.WristGoToTarget;
import frc.robot.subsystems.extensor.Lift;
import frc.robot.subsystems.extensor.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlgaeSafeWristLiftDown extends InstantCommand {
  Lift lift;
  Wrist wrist;

  boolean goingUp;
  double liftTarget;
  double wristTarget;

  public AlgaeSafeWristLiftDown() {
    wrist = Robot.getRobotContainer().getWrist();
    lift = Robot.getRobotContainer().getLift();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(wrist, lift);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if (Robot.isCoralMode()) {
      return;
    } else {
      liftTarget = HighAltitudeConstants.LIFT_ALGAE_POSITIONS[1];
      wristTarget = 90;
    }
    (new SequentialCommandGroup(new WristGoToTarget(wristTarget),
        new LiftGoToTarget(HighAltitudeConstants.LIFT_MAX_POWER, liftTarget))).schedule();

  }
}
