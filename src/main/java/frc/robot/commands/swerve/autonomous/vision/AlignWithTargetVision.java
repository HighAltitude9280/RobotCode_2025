// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve.autonomous.vision;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.HighAltitudeConstants;
import frc.robot.Robot;
import frc.robot.HighAltitudeConstants.REEF_POSITION;
import frc.robot.HighAltitudeConstants.REEF_SIDE;;

public class AlignWithTargetVision extends Command {

  boolean isFinished = false;

  int targetID;
  double targetAngle;
  double targetYaw;

  double maxTurnPower, maxSpeedPower, maxStrafePower;

  /**
   * Command to align the robot using vision to the desired reef branch.
   * Note that this command automatically accounts for the alliance.
   * 
   * @param position       The position to align the robot to.
   * @param left           True for left branch, false for left
   * @param maxTurnPower   Max turning power (from 0 to 1).
   * @param maxSpeedPower  Max speed power (from 0 to 1).
   * @param maxStrafePower Max strafe power (from 0 to 1).
   */
  public AlignWithTargetVision(REEF_POSITION position, boolean left, double maxTurnPower, double maxSpeedPower,
      double maxStrafePower) {
    addRequirements(Robot.getRobotContainer().getSwerveDriveTrain());

    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      this.targetID = HighAltitudeConstants.RED_APRILTAG_IDS[position.getID()];
      this.targetAngle = HighAltitudeConstants.PATHFINDING_REEF_POS[position.getID()].getRotation().getDegrees();
    } else {
      this.targetID = HighAltitudeConstants.BLUE_APRILTAG_IDS[position.getID()];
      this.targetAngle = HighAltitudeConstants.PATHFINDING_REEF_POS[position.getID()].getRotation().getDegrees();
    }
    if (left) {
      targetYaw = HighAltitudeConstants.VISION_YAW_OFFSET_TARGET_LEFT;
    } else {
      targetYaw = HighAltitudeConstants.VISION_YAW_OFFSET_TARGET_RIGHT;
    }
    this.maxTurnPower = maxTurnPower;
    this.maxSpeedPower = maxSpeedPower;
    this.maxStrafePower = maxStrafePower;
  }

  /**
   * Command to align the robot using vision to the desired reef branch, based on
   * the robot modes (front/back, left/right).
   * 
   * Note that this command automatically accounts for the alliance.
   * 
   * @param side           The side of the reef to align the robot (left, center,
   *                       or right).
   * @param maxTurnPower   Max turning power.
   * @param maxSpeedPower  Max speed power.
   * @param maxStrafePower Max strafe power.
   */
  public AlignWithTargetVision(REEF_SIDE side, double maxTurnPower, double maxSpeedPower,
      double maxStrafePower) {
    this(side.getPosition(Robot.isFrontMode()), Robot.isLeftMode(),
        maxTurnPower, maxSpeedPower, maxStrafePower);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double yaw = Robot.getRobotContainer().getVision().getTargetYaw(targetID);
    double area = Robot.getRobotContainer().getVision().getTargetSize(targetID);

    yaw = yaw == Double.NaN ? targetYaw : yaw;
    area = area == Double.NaN ? HighAltitudeConstants.VISION_AREA_TARGET : area;

    isFinished = Robot.getRobotContainer().getSwerveDriveTrain().alignWithTarget(targetAngle, yaw, area, targetYaw,
        HighAltitudeConstants.VISION_AREA_TARGET, maxTurnPower, maxStrafePower, maxStrafePower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.getRobotContainer().getSwerveDriveTrain().stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
