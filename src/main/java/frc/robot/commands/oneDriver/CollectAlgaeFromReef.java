// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.oneDriver;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.HighAltitudeConstants;
import frc.robot.HighAltitudeConstants.REEF_HEIGHT;
import frc.robot.commands.extensor.compound.both.LiftWristGoToTargetHeight;
import frc.robot.commands.swerve.autonomous.SwerveMoveMeters;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CollectAlgaeFromReef extends SequentialCommandGroup {
  REEF_HEIGHT height;

  /** Creates a new CollectAlgaeFromReef. */
  public CollectAlgaeFromReef(boolean algaeUp) {
    if (algaeUp)
      height = REEF_HEIGHT.L3;
    else
      height = REEF_HEIGHT.L2;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new DriveToPose(new Pose2d(2.8, 4.0, Rotation2d.fromDegrees(0))),
        new SwerveMoveMeters(0.4, 0, 0.5));
  }
}
