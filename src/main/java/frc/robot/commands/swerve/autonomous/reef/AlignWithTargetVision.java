// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve.autonomous.reef;

import com.fasterxml.jackson.annotation.Nulls;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.HighAltitudeConstants;
import frc.robot.Robot;
import frc.robot.HighAltitudeConstants.REEF_POSITION;
import frc.robot.HighAltitudeConstants.REEF_SIDE;;

public class AlignWithTargetVision extends Command 
{
  private boolean isFinished = false;
  private int targetID = -1;
  private double targetAngle = Double.NaN;
  private double targetYaw;
  private REEF_POSITION pos;
  private REEF_SIDE side;
  private Boolean left;
  private final double maxTurnPower, maxSpeedPower, maxStrafePower;

  /**
   * Command to align the robot using vision.
   *
   * @param position      The position to align the robot to (nullable for detection mode).
   * @param side          The reef side (nullable for detection mode).
   * @param left          True for left branch, false for right (nullable for detection mode).
   * @param maxTurnPower  Max turning power.
   * @param maxSpeedPower Max speed power.
   * @param maxStrafePower Max strafe power.
   */
  public AlignWithTargetVision(REEF_POSITION position, REEF_SIDE side, Boolean left,
                               double maxTurnPower, double maxSpeedPower, double maxStrafePower) 
  {
    addRequirements(Robot.getRobotContainer().getSwerveDriveTrain());

    this.pos = position;
    this.left = left;
    this.maxTurnPower = maxTurnPower;
    this.maxSpeedPower = maxSpeedPower;
    this.maxStrafePower = maxStrafePower;
    this.side = side;
  }
  
  @Override
  public void initialize() {
    if (pos == null && side != null)
      this.pos = side.getPosition(Robot.isFrontMode());
    
    left = left != null ? left : Robot.isLeftMode();
    Robot.getRobotContainer().getSwerveDriveTrain().setIsFieldOriented(false);
    determineTarget();
  }

  private void determineTarget() 
  {
    targetYaw = left ? HighAltitudeConstants.VISION_YAW_OFFSET_TARGET_LEFT
                     : HighAltitudeConstants.VISION_YAW_OFFSET_TARGET_RIGHT;

    var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);

    int[] reefIDs = (alliance == DriverStation.Alliance.Red)
        ? HighAltitudeConstants.RED_APRILTAG_IDS
        : HighAltitudeConstants.BLUE_APRILTAG_IDS;

    Pose2d[] reefPoses = (alliance == DriverStation.Alliance.Red)
        ? HighAltitudeConstants.PATHFINDING_RED_REEF_POS
        : HighAltitudeConstants.PATHFINDING_BLUE_REEF_POS;

    if (pos == null) 
    {
      targetID = Robot.getRobotContainer().getVision().getTargetID();
      if(targetID == -1) return;

      for (int i = 0; i < reefIDs.length; i++) {
        if (targetID == reefIDs[i]) 
        {
          targetAngle = reefPoses[i].getRotation().getDegrees();
          break;
        }
      }
    } 
    else 
    {
      targetID = reefIDs[pos.getID()];
      targetAngle = reefPoses[pos.getID()].getRotation().getDegrees();
    }
  }

  @Override
  public void execute() {
    if (Double.isNaN(targetAngle)) 
    {
      determineTarget();
      if (Double.isNaN(targetAngle)) return;
    }
    
    double yaw = Robot.getRobotContainer().getVision().getTargetYaw(targetID);
    double area = Robot.getRobotContainer().getVision().getTargetSize(targetID);

    yaw = Double.isNaN(yaw) ? targetYaw : yaw;
    area = Double.isNaN(area) ? HighAltitudeConstants.VISION_AREA_TARGET : area;

    System.out.println("TargetAngle: " + targetAngle);

    isFinished = Robot.getRobotContainer().getSwerveDriveTrain().alignWithTarget(
        targetAngle, yaw, area, targetYaw,
        HighAltitudeConstants.VISION_AREA_TARGET,
        maxTurnPower, maxSpeedPower, maxStrafePower);
  }

  @Override
  public void end(boolean interrupted) {
    Robot.getRobotContainer().getSwerveDriveTrain().stopModules();
    Robot.getRobotContainer().getSwerveDriveTrain().setIsFieldOriented(true);
  }

  @Override
  public boolean isFinished() {
    return isFinished;
  }
}

