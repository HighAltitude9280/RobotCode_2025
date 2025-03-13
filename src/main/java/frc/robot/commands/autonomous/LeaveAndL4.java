// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.HighAltitudeConstants.REEF_HEIGHT;
import frc.robot.commands.modes.SetCoralMode;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LeaveAndL4 extends SequentialCommandGroup {
  /** Creates a new LeaveAndL4. */
  public LeaveAndL4() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new SetCoralMode(true),
        new AutoLeave(2.2, 0.7).withTimeout(3),
        new ScoreCoral(REEF_HEIGHT.TOP));
  }

}
