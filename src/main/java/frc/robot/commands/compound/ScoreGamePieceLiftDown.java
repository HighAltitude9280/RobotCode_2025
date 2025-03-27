// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.compound;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.HighAltitudeConstants;
import frc.robot.Robot;
import frc.robot.HighAltitudeConstants.REEF_HEIGHT;
import frc.robot.commands.extensor.gripper.manual.ScoreGamePiece;
import frc.robot.commands.extensor.lift.control.LiftDefaultCommand;
import frc.robot.commands.swerve.autonomous.SwerveMoveMeters;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreGamePieceLiftDown extends SequentialCommandGroup {
  /** Creates a new ScoreCoralLiftDown. */
  public ScoreGamePieceLiftDown() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ParallelRaceGroup(
            new ScoreGamePiece(HighAltitudeConstants.GRIPPER_IN_SPEED), new WaitCommand(0.5),
            new LiftDefaultCommand(HighAltitudeConstants.LIFT_MAX_POWER, HighAltitudeConstants.LIFT_ARRIVE_OFFSET)),
        new SwerveMoveMeters(-0.2, Robot.getRobotContainer().getSwerveDriveTrain().getHeadingCCWPositive(),
            0.2),
        new LiftWristGoToTargetHeight(REEF_HEIGHT.BOTTOM));
  }
}
