// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.compound;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.HighAltitudeConstants;
import frc.robot.Robot;

public class LiftWristIntakeAlgae extends Command {
  public LiftWristIntakeAlgae() 
  {
  }

  @Override
  public void initialize() 
  {
    Robot.getRobotContainer().getLift().setTarget(HighAltitudeConstants.LIFT_ALGAE_INTAKE_POSITION);
    Robot.getRobotContainer().getWrist().setTarget(HighAltitudeConstants.WRIST_ALGAE_INTAKE_POSITION);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return Robot.getRobotContainer().getWrist().onTarget() && Robot.getRobotContainer().getLift().onTarget();
  }
}
