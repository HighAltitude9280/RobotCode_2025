// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.oneDriver;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Robot;
import frc.robot.HighAltitudeConstants.REEF_HEIGHT;
import frc.robot.commands.extensor.gripper.manual.IntakeAlgae;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OneDriverPro extends InstantCommand {
  boolean left;
  REEF_HEIGHT height;
  boolean up;

  public OneDriverPro(boolean left, REEF_HEIGHT height, boolean up) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.height = height;
    this.left = left;
    this.up = up;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (Robot.isCoralMode()) {
      new AlignWithBranchAndScore(left, height);
    } else {
      new ParallelCommandGroup(new CollectAlgaeFromReef(up), new IntakeAlgae());
    }

  }
}
