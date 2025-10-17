// src/main/java/frc/robot/commands/swerve/autonomous/offSeason/DriveToCoralStation.java
package frc.robot.commands.swerve.autonomous.offSeason;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.HighAltitudeConstantsPose;
import frc.robot.HighAltitudeConstantsPose.CORAL_STATION_POSITION;
import frc.robot.Robot;

public class DriveToCoralStation extends Command {

  // ---------------- Parameters ----------------
  private final boolean l1Mode; // <--- L1 ON/OFF
  private CORAL_STATION_POSITION pos;

  /** true = left, false = right; null = AUTO (elige el más cercano) */
  private final Boolean left;

  private final double maxLinearVelocity;
  private final double maxAngularVelocity;

  // ---------------- State ----------------
  private Pose2d targetPose;
  private boolean done;

  // ---------------- Constructors ----------------

  /** Constructor general con L1. */
  public DriveToCoralStation(boolean L1, CORAL_STATION_POSITION position, Boolean left,
      double maxLinearVelocity, double maxAngularVelocity) {
    addRequirements(Robot.getRobotContainer().getSwerveDriveTrain());
    this.l1Mode = L1;
    this.pos = position;
    this.left = left;
    this.maxLinearVelocity = maxLinearVelocity;
    this.maxAngularVelocity = maxAngularVelocity;
  }

  /** Conveniencia: sólo L/R, MIDDLE por defecto (soporta L1). */
  public DriveToCoralStation(boolean L1, boolean left, double maxLinearVelocity,
      double maxAngularVelocity) {
    this(L1, CORAL_STATION_POSITION.MIDDLE, Boolean.valueOf(left), maxLinearVelocity,
        maxAngularVelocity);
  }

  // ---------------- Lifecycle ----------------

  @Override
  public void initialize() {
    done = false;
    if (pos == null)
      pos = CORAL_STATION_POSITION.MIDDLE;

    targetPose = l1Mode ? pickL1Pose() : pickStationPose();

    if (targetPose == null) {
      System.err.println("[DriveToCoralStation] targetPose is null, fallback (0,0,0).");
      targetPose = new Pose2d();
    }
    pushDebug();
  }

  @Override
  public void execute() {
    done = Robot.getRobotContainer().getSwerveDriveTrain().AlignWithTargetPose(targetPose,
        maxLinearVelocity, maxAngularVelocity);
  }

  @Override
  public void end(boolean interrupted) {
    Robot.getRobotContainer().getSwerveDriveTrain().stopModules();
  }

  @Override
  public boolean isFinished() {
    return done;
  }

  // ---------------- Internals ----------------

  /** L1: elige entre {BLUE_LEFT, BLUE_RIGHT} o {RED_LEFT, RED_RIGHT}. */
  private Pose2d pickL1Pose() {
    var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);

    Pose2d candLeft = null, candRight = null;
    switch (alliance) {
      case Red:
        candLeft = HighAltitudeConstantsPose.CORAL_STATION_L1_RED_LEFT;
        candRight = HighAltitudeConstantsPose.CORAL_STATION_L1_RED_RIGHT;
        break;
      case Blue:
      default:
        candLeft = HighAltitudeConstantsPose.CORAL_STATION_L1_BLUE_LEFT;
        candRight = HighAltitudeConstantsPose.CORAL_STATION_L1_BLUE_RIGHT;
        break;
    }

    if (candLeft == null && candRight == null)
      return null;

    // Forzado por parámetro
    if (left != null) {
      return left.booleanValue() ? (candLeft != null ? candLeft : candRight)
          : (candRight != null ? candRight : candLeft);
    }

    // Auto: el más cercano a la pose actual
    Pose2d current = Robot.getRobotContainer().getSwerveDriveTrain().getPose();
    if (current == null)
      return (candLeft != null) ? candLeft : candRight;

    double dL = (candLeft != null) ? candLeft.getTranslation().getDistance(current.getTranslation())
        : Double.MAX_VALUE;
    double dR =
        (candRight != null) ? candRight.getTranslation().getDistance(current.getTranslation())
            : Double.MAX_VALUE;

    return (dL <= dR) ? candLeft : candRight;
  }

  /** Lógica original (no L1): tablas por alianza, side forzado o AUTO. */
  private Pose2d pickStationPose() {
    int idx = pos.getID();
    var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);

    Pose2d candLeft = null, candRight = null;

    switch (alliance) {
      case Red:
        if (idx < HighAltitudeConstantsPose.PATHFINDING_RED_LEFT_CORAL_STATION.length)
          candLeft = HighAltitudeConstantsPose.PATHFINDING_RED_LEFT_CORAL_STATION[idx];
        if (idx < HighAltitudeConstantsPose.PATHFINDING_RED_RIGHT_CORAL_STATION.length)
          candRight = HighAltitudeConstantsPose.PATHFINDING_RED_RIGHT_CORAL_STATION[idx];
        break;

      case Blue:
      default:
        if (idx < HighAltitudeConstantsPose.PATHFINDING_BLUE_LEFT_CORAL_STATION.length)
          candLeft = HighAltitudeConstantsPose.PATHFINDING_BLUE_LEFT_CORAL_STATION[idx];
        if (idx < HighAltitudeConstantsPose.PATHFINDING_BLUE_RIGHT_CORAL_STATION.length)
          candRight = HighAltitudeConstantsPose.PATHFINDING_BLUE_RIGHT_CORAL_STATION[idx];
        break;
    }

    if (candLeft == null && candRight == null)
      return null;

    if (left != null) {
      return left.booleanValue() ? (candLeft != null ? candLeft : candRight)
          : (candRight != null ? candRight : candLeft);
    }

    Pose2d current = Robot.getRobotContainer().getSwerveDriveTrain().getPose();
    if (current == null)
      return (candLeft != null) ? candLeft : candRight;

    double dL = (candLeft != null) ? candLeft.getTranslation().getDistance(current.getTranslation())
        : Double.MAX_VALUE;
    double dR =
        (candRight != null) ? candRight.getTranslation().getDistance(current.getTranslation())
            : Double.MAX_VALUE;

    return (dL <= dR) ? candLeft : candRight;
  }

  private void pushDebug() {
    if (targetPose == null)
      return;
    SmartDashboard.putBoolean("CoralStation/L1Mode", l1Mode);
    SmartDashboard.putNumber("CoralStation/TargetPoseX", targetPose.getX());
    SmartDashboard.putNumber("CoralStation/TargetPoseY", targetPose.getY());
    SmartDashboard.putNumber("CoralStation/TargetPoseDeg", targetPose.getRotation().getDegrees());
    SmartDashboard.putString("CoralStation/Alliance",
        DriverStation.getAlliance().map(Enum::name).orElse("Unknown"));
    SmartDashboard.putString("CoralStation/PosIndex", pos.name());
    SmartDashboard.putString("CoralStation/Side",
        left == null ? "AUTO" : (left.booleanValue() ? "LEFT" : "RIGHT"));
  }
}
