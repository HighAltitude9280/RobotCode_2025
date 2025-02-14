// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve.autonomous.reef;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.HighAltitudeConstants;
import frc.robot.HighAltitudeConstants.REEF_POSITION;
import frc.robot.HighAltitudeConstants.REEF_SIDE;
import frc.robot.subsystems.swerve.SwerveDriveTrain;
import frc.robot.Robot;

public class PathplanToReefThenVision extends InstantCommand 
{
  REEF_POSITION pos = null;
  REEF_SIDE side;

  boolean left;

  double maxTurnPower, maxSpeedPower, maxStrafePower;

  public PathplanToReefThenVision(REEF_POSITION pos, boolean left, double maxTurnPower, double maxSpeedPower, double maxStrafePower) 
  {
    this.pos = pos;
    this.left = left;

    this.maxTurnPower = maxTurnPower;
    this.maxSpeedPower = maxSpeedPower;
    this.maxStrafePower = maxStrafePower;
  }
  public PathplanToReefThenVision(REEF_SIDE side, double maxTurnPower, double maxSpeedPower, double maxStrafePower) 
  {
    this.side = side;
    this.maxTurnPower = maxTurnPower;
    this.maxSpeedPower = maxSpeedPower;
    this.maxStrafePower = maxStrafePower;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    Command align;
    if(pos == null)
    {
      pos = side.getPosition(Robot.isFrontMode());
      align = new AlignWithTargetVision(side, maxTurnPower, maxSpeedPower, maxStrafePower);
    }
    else
    align = new AlignWithTargetVision(pos, left, maxTurnPower, maxSpeedPower, maxStrafePower);

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
