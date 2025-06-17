// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.extensor.compound.notbeingused;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.HighAltitudeConstants;
import frc.robot.HighAltitudeConstants.REEF_HEIGHT;
import frc.robot.commands.extensor.compound.both.LiftWristGoToTargetHeight;
import frc.robot.commands.extensor.lift.control.LiftGoToTarget;
import frc.robot.commands.extensor.wrist.control.WristGoToTarget;
import frc.robot.commands.modes.SetCoralMode;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CoralModeLiftWrist extends SequentialCommandGroup {
  /** Creates a new CoralModeLiftWrist. */
  double wristTarget = 0;

  public CoralModeLiftWrist(boolean coralMode) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    if (coralMode)
      wristTarget = HighAltitudeConstants.WRIST_ZERO_ANGLE;
    else
      wristTarget = HighAltitudeConstants.WRIST_ALGAE_POSITION;

    addCommands(
        new LiftGoToTarget(HighAltitudeConstants.LIFT_MAX_POWER, HighAltitudeConstants.LIFT_TRANSITION_POSITION),
        new WristGoToTarget(wristTarget, HighAltitudeConstants.WRIST_DRIVE_SPEED), new SetCoralMode(coralMode),
        new LiftWristGoToTargetHeight(REEF_HEIGHT.BOTTOM));
  }
}
