// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve.autonomous.reef;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.HighAltitudeConstants;
import frc.robot.HighAltitudeConstants.REEF_POSITION;
import frc.robot.HighAltitudeConstants.REEF_SIDE;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.SwerveDriveTrain;

public class PathplanToReefThenVisionPose extends InstantCommand 
{
  REEF_POSITION pos = null;
  REEF_SIDE side;

  Boolean left;

  double maxLinearVelocity, maxAngularVelocity;

  public PathplanToReefThenVisionPose(REEF_POSITION pos, REEF_SIDE side, Boolean left, double maxLinearVelocity, double maxAngularVelocity) 
  {
    this.pos = pos;
    this.left = left;
    this.side = side;

    this.maxAngularVelocity = maxAngularVelocity;
    this.maxLinearVelocity = maxLinearVelocity;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    if (pos == null)
    {
      if(side == null) 
        side = Robot.getReefMode();
      pos = side.getPosition(Robot.isFrontMode());
    }
    var align = new AlignWithTargetPose(pos, side, left,maxLinearVelocity, maxAngularVelocity);

    Pose2d targetPose;
    var alliance = DriverStation.getAlliance();

    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red)
      targetPose = HighAltitudeConstants.PATHFINDING_RED_REEF_POS[pos.getID()]; 
    else
      targetPose = HighAltitudeConstants.PATHFINDING_BLUE_REEF_POS[pos.getID()];

    var path = SwerveDriveTrain.pathfindToPose(targetPose);

    new SequentialCommandGroup(path, align).schedule();
  }
}
