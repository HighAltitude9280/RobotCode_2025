// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.HighAltitudeConstants;
import frc.robot.HighAltitudeConstants.REEF_POSITION;
import frc.robot.HighAltitudeConstants.REEF_SIDE;
import frc.robot.commands.swerve.autonomous.reef.AlignWithTargetVision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignVisionMoveMeters extends SequentialCommandGroup {
  /** Creates a new AlignVisionMoveMeters. */
  public AlignVisionMoveMeters(REEF_POSITION pos, REEF_SIDE side, Boolean left) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new AlignWithTargetVision(pos, side, left, HighAltitudeConstants.VISION_TURN_MAX_POWER,
        HighAltitudeConstants.VISION_SPEED_MAX_POWER,
        HighAltitudeConstants.VISION_STRAFE_MAX_POWER), new TurnWheels(0),
        new SwerveMoveMeters(HighAltitudeConstants.SWERVE_METERS_DISTANCE_ALIGN_TO_REEF, 0,
            HighAltitudeConstants.VISION_SPEED_MAX_POWER));
  }
}
