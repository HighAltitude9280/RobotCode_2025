// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.oneDriver;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.HighAltitudeConstants;
import frc.robot.HighAltitudeConstants.REEF_HEIGHT;
import frc.robot.commands.autonomous.ScoreCoral;
import frc.robot.commands.extensor.compound.both.LiftWristGoToTargetHeight;
import frc.robot.commands.extensor.compound.coral.ScoreGamePieceLiftDown;
import frc.robot.commands.swerve.autonomous.reef.AlignWithTargetPose;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignWithBranchAndScore extends SequentialCommandGroup {
  /** Creates a new AlignWithRightBranchAndScore. */
  public AlignWithBranchAndScore(boolean left, REEF_HEIGHT height) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

        new AlignWithTargetPose(null, null, left,
            HighAltitudeConstants.VISION_POSE_MAX_SPEED,
            HighAltitudeConstants.VISION_POSE_MAX_TURN),
        new ScoreCoral(height)

    );
  }
}
