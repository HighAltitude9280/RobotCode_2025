package frc.robot.commands.swerve.autonomous.offSeason;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.HighAltitudeConstantsPose;
import frc.robot.Robot;
import frc.robot.resources.math.PoseUtil;
import frc.robot.subsystems.swerve.SwerveDriveTrain;

/**
 * Drives to the NET approach pose (alliance-aware).
 * Trusts the pre-baked Pose2d. Optional local forward/backoff offset.
 */
public class DriveToNet extends Command {

  private final double vMax,

      wMax;
  private final double offsetMeters;
  private final boolean enableTelemetry;

  private SwerveDriveTrain swerve;
  private Pose2d targetPose;
  private boolean done;

  /**
   * Simple ctor: no offset, no telemet
   * ry.
   */
  public DriveToNet(double maxLinearVelocity, double maxAngularVelocity) {
    this(maxLinearVelocity, maxAngularVelocity, 0.0, false);
  }

  /** Full ctor: allows offset and telemetry toggle. */
  public DriveToNet(double maxLinearVelocity, double maxAngularVelocity,
      double offsetMeters, boolean enableTelemetry) {
    this.vMax = maxLinearVelocity;
    this.wMax = maxAngularVelocity;
    this.offsetMeters = offsetMeters;
    this.enableTelemetry = enableTelemetry;
    addRequirements(Robot.getRobotContainer().getSwerveDriveTrain());
  }

  @Override
  public void initialize() {
    swerve = Robot.getRobotContainer().getSwerveDriveTrain();
    done = false;

    var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
    targetPose = (alliance == DriverStation.Alliance.Red)
        ? HighAltitudeConstantsPose.PATHFINDING_RED_NET
        : HighAltitudeConstantsPose.PATHFINDING_BLUE_NET;

    if (targetPose == null)
      targetPose = new Pose2d(); // safety fallback

    // Apply local forward/backoff offset if requested
    if (Math.abs(offsetMeters) > 1e-6) {
      targetPose = PoseUtil.offsetLocal(targetPose, offsetMeters, 0.0);
    }

    if (enableTelemetry)
      pushTelemetry();
  }

  @Override
  public void execute() {
    done = swerve.AlignWithTargetPose(targetPose, vMax, wMax);
  }

  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();
  }

  @Override
  public boolean isFinished() {
    return done;
  }

  /** Telemetry is opt-in to avoid dashboard noise. */
  private void pushTelemetry() {
    // SmartDashboard.putNumber("NET/X", targetPose.getX());
    // SmartDashboard.putNumber("NET/Y", targetPose.getY());
    // SmartDashboard.putNumber("NET/deg", targetPose.getRotation().getDegrees());
  }
}
