// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.modes;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetCoralMode extends InstantCommand 
{
  private boolean coraMode;

  public SetCoralMode(boolean coralMode) 
  {
    this.coraMode = coralMode;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    Robot.setCoralMode(coraMode);
  }
}
