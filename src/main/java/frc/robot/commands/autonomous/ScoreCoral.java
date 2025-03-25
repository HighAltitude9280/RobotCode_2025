// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.HighAltitudeConstants;
import frc.robot.HighAltitudeConstants.REEF_HEIGHT;
import frc.robot.commands.extensor.gripper.manual.ScoreGamePiece;
import frc.robot.commands.extensor.lift.control.LiftDefaultCommand;
import frc.robot.commands.extensor.lift.control.LiftGoToTarget;
import frc.robot.commands.extensor.wrist.control.WristGoToTarget;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreCoral extends SequentialCommandGroup {

    /** Creates a new ScoreL4. */
    public ScoreCoral(REEF_HEIGHT heightUp) {
        double liftTargetUp = HighAltitudeConstants.LIFT_CORAL_POSITIONS[heightUp.getID()];
        double wristTargetUp = HighAltitudeConstants.WRIST_CORAL_POSITIONS[heightUp.getID()];

        REEF_HEIGHT heightDown = REEF_HEIGHT.BOTTOM;
        double liftTargetDown = HighAltitudeConstants.LIFT_CORAL_POSITIONS[heightDown.getID()];
        double wristTargetDown = HighAltitudeConstants.WRIST_CORAL_POSITIONS[heightDown.getID()];

        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                new LiftGoToTarget(HighAltitudeConstants.LIFT_MAX_POWER, liftTargetUp,
                        HighAltitudeConstants.LIFT_ARRIVE_OFFSET),
                new ParallelRaceGroup(
                        new LiftDefaultCommand(HighAltitudeConstants.LIFT_MAX_POWER,
                                HighAltitudeConstants.LIFT_ARRIVE_OFFSET),
                        new WristGoToTarget(wristTargetUp, HighAltitudeConstants.WRIST_DRIVE_SPEED)),

                new ScoreGamePiece(HighAltitudeConstants.GRIPPER_IN_SPEED).withTimeout(0.5),

                new ParallelRaceGroup(
                        new LiftDefaultCommand(HighAltitudeConstants.LIFT_MAX_POWER,
                                HighAltitudeConstants.LIFT_ARRIVE_OFFSET),
                        new WristGoToTarget(wristTargetDown, HighAltitudeConstants.WRIST_DRIVE_SPEED)),
                new LiftGoToTarget(HighAltitudeConstants.LIFT_MAX_POWER, liftTargetDown,
                        HighAltitudeConstants.LIFT_ARRIVE_OFFSET));
    }
}
