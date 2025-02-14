// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve.autonomous.reef;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.HighAltitudeConstants;
import frc.robot.HighAltitudeConstants.REEF_POSITION;
import frc.robot.HighAltitudeConstants.REEF_SIDE;
import frc.robot.subsystems.swerve.SwerveDriveTrain;
import frc.robot.Robot;

public class PathplanToReef extends InstantCommand 
{
  REEF_POSITION pos = null;

  REEF_SIDE side;

  public PathplanToReef(REEF_POSITION pos) 
  {
    this.pos = pos;
  }
  public PathplanToReef(REEF_SIDE side) 
  {
    this.side = side;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    if(pos == null)
      pos = side.getPosition(Robot.isFrontMode());

    Pose2d targetPose;
    var alliance = DriverStation.getAlliance();

    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red)
      targetPose = HighAltitudeConstants.PATHFINDING_RED_REEF_POS[pos.getID()]; 
    else
      targetPose = HighAltitudeConstants.PATHFINDING_BLUE_REEF_POS[pos.getID()];

    SwerveDriveTrain.pathfindToPose(targetPose).schedule();
  }
}
