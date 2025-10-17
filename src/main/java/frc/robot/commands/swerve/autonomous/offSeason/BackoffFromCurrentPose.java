// src/main/java/frc/robot/commands/swerve/autonomous/offSeason/BackoffFromCurrentPose.java
package frc.robot.commands.swerve.autonomous.offSeason;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.resources.math.PoseUtil;
import frc.robot.subsystems.swerve.SwerveDriveTrain;

public class BackoffFromCurrentPose extends Command {
  private final double backoffMeters, vMax, wMax;
  private SwerveDriveTrain swerve;
  private Pose2d goal;
  private boolean reached;

  /**
   * Retrocede backoffMeters en el marco local del robot (desde la pose actual).
   * 
   * @param backoffMeters
   * @param vMax use HighAltitudeConstants.VISION_POSE_MAX_SPEED;
   * @param wMax use HighAltitudeconstants.VISION_POSE_MAX_TURN;
   */
  public BackoffFromCurrentPose(double backoffMeters, double vMax, double wMax) {
    this.backoffMeters = backoffMeters;
    this.vMax = vMax;
    this.wMax = wMax;
    addRequirements(Robot.getRobotContainer().getSwerveDriveTrain());
  }

  @Override
  public void initialize() {
    swerve = Robot.getRobotContainer().getSwerveDriveTrain();
    goal = PoseUtil.backoff(swerve.getPose(), backoffMeters);
    reached = false;
  }

  @Override
  public void execute() {
    reached = swerve.AlignWithTargetPose(goal, vMax, wMax);
  }

  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();
  }

  @Override
  public boolean isFinished() {
    return reached;
  }
}
