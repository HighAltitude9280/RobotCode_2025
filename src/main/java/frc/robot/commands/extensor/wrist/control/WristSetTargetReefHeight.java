// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.extensor.wrist.control;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.HighAltitudeConstants;
import frc.robot.Robot;
import frc.robot.HighAltitudeConstants.REEF_HEIGHT;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class WristSetTargetReefHeight extends InstantCommand {
  REEF_HEIGHT height;
  public WristSetTargetReefHeight(REEF_HEIGHT height) 
  {
    this.height = height;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    if(Robot.isCoralMode())
      Robot.getRobotContainer().getWrist().setTarget(HighAltitudeConstants.WRIST_CORAL_POSITIONS[height.getID()]);
    else
      Robot.getRobotContainer().getWrist().setTarget(HighAltitudeConstants.WRIST_ALGAE_POSITIONS[height.getID()]);
  }
}
