// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.extensor.compound.algae;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import frc.robot.HighAltitudeConstants.REEF_HEIGHT;
import frc.robot.commands.extensor.compound.both.LiftWristGoToTargetHeight;
import frc.robot.commands.extensor.gripper.manual.HoldAlgae;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class KeepAlgaeSafeLiftDown extends ParallelCommandGroup {
  /** Creates a new KeepAlgaeSafeLiftDown. */
  public KeepAlgaeSafeLiftDown() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new HoldAlgae(),
        new LiftWristGoToTargetHeight(REEF_HEIGHT.BOTTOM));
  }
}
