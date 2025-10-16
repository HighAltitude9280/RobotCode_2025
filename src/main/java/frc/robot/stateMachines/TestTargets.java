package frc.robot.stateMachines;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.HighAltitudeConstants;
import frc.robot.HighAltitudeConstantsPose;
import frc.robot.HighAltitudeConstantsPose.CORAL_STATION_POSITION;
import frc.robot.Robot;
import frc.robot.Robot.GameMode;

public final class TestTargets {
  private static volatile int caralho_var = 0; // L1/LX: 0..3 (A/B,C/D,E/F,G/H) | ALGAE: 0..5
  // (BC,BR,FR,FC,FL,BL)
  private static volatile boolean TELEMETRY = true;

  private TestTargets() {}

  // --- Public API ------------------------------------------------------------

  public static void setCaralho_var(int v) {
    caralho_var = MathUtil.clamp(v, 0, 5);
  }

  public static int getCaralho_var() {
    return caralho_var;
  }

  public static void enableTelemetry(boolean on) {
    TELEMETRY = on;
  }

  /** Returns the test target pose based on current GameMode + side + variant. */
  public static Pose2d currentPose() {
    GameMode mode = Robot.getRobotContainer().getGameMode();
    switch (mode) {
      case CORAL_L1:
        // Exact branch (A..H) with NO backoff
        return selectBranchPose(/* pair */ MathUtil.clamp(caralho_var, 0, 3));

      case CORAL_LX:
        // Branch (A..H) with backoff ALWAYS "behind" the target orientation
        return backoffAlongHeading(selectBranchPose(/* pair */ MathUtil.clamp(caralho_var, 0, 3)),
            Math.abs(HighAltitudeConstants.CORAL_BACKOFF_M));

      case ALGAE:
        // Tag → face center + retract behind face heading
        return algaeRemovalPose();

      case MANUAL:
      default:
        return new Pose2d(); // no-op
    }
  }

  /** Coral Station position from variant (0..2 => NEAR/MIDDLE/FAR). */
  public static CORAL_STATION_POSITION coralStationPosFromVariant() {
    int idx = MathUtil.clamp(caralho_var, 0, 2);
    return switch (idx) {
      case 0 -> CORAL_STATION_POSITION.NEAR;
      case 1 -> CORAL_STATION_POSITION.MIDDLE;
      case 2 -> CORAL_STATION_POSITION.FAR;
      default -> CORAL_STATION_POSITION.MIDDLE;
    };
  }

  // --- Internal helpers ------------------------------------------------------

  /** Chooses branch A..H by (pair 0..3) and left/right (A,C,E,G vs B,D,F,H), alliance-aware. */
  private static Pose2d selectBranchPose(int pair) {
    boolean leftBranch = Robot.isLeftMode(); // true => A,C,E,G ; false => B,D,F,H
    int idx = switch (pair) {
      case 0 -> leftBranch ? 0 : 1; // A/B
      case 1 -> leftBranch ? 2 : 3; // C/D
      case 2 -> leftBranch ? 4 : 5; // E/F
      case 3 -> leftBranch ? 6 : 7; // G/H
      default -> 0;
    };
    Alliance al = DriverStation.getAlliance().orElse(Alliance.Blue);
    return (al == Alliance.Red) ? HighAltitudeConstantsPose.PATHFINDING_RED_BRANCHES[idx]
        : HighAltitudeConstantsPose.PATHFINDING_BLUE_BRANCHES[idx];
  }

  /** Returns reef face (BC..BL) from variant 0..5, alliance-aware. */
  private static Pose2d selectReefFace(int faceIdx) {
    faceIdx = MathUtil.clamp(faceIdx, 0, 5); // 0=BC 1=BR 2=FR 3=FC 4=FL 5=BL
    Alliance al = DriverStation.getAlliance().orElse(Alliance.Blue);
    return (al == Alliance.Red) ? HighAltitudeConstantsPose.PATHFINDING_RED_REEF_POS[faceIdx]
        : HighAltitudeConstantsPose.PATHFINDING_BLUE_REEF_POS[faceIdx];
  }

  /**
   * ALGAE: pick face center from detected AprilTag (level & face), then apply retract. Tag mapping
   * (by your spec): Blue L3: 18→FC(3), 22→BR(1), 20→BL(5) Blue L2: 19→FL(4), 17→FR(2), 21→BC(0) Red
   * L3: 7→FC(3), 9→BR(1), 11→BL(5) Red L2: 6→FL(4), 8→FR(2), 10→BC(0)
   */
  private static Pose2d algaeRemovalPose() {
    int tag = Robot.getRobotContainer().getVision().getTargetID();
    Alliance al = DriverStation.getAlliance().orElse(Alliance.Blue);

    int reefIdx = -1; // 0=BC 1=BR 2=FR 3=FC 4=FL 5=BL
    if (tag > 0) {
      if (al == Alliance.Blue) {
        if (tag == 18)
          reefIdx = 3; // FC
        else if (tag == 22)
          reefIdx = 1; // BR
        else if (tag == 20)
          reefIdx = 5; // BL
        else if (tag == 19)
          reefIdx = 4; // FL
        else if (tag == 17)
          reefIdx = 2; // FR
        else if (tag == 21)
          reefIdx = 0; // BC
      } else { // Red
        if (tag == 7)
          reefIdx = 3; // FC
        else if (tag == 9)
          reefIdx = 1; // BR
        else if (tag == 11)
          reefIdx = 5; // BL
        else if (tag == 6)
          reefIdx = 4; // FL
        else if (tag == 8)
          reefIdx = 2; // FR
        else if (tag == 10)
          reefIdx = 0; // BC
      }
    }

    Pose2d base;
    boolean mapped = (reefIdx >= 0);
    if (mapped) {
      base = selectReefFace(reefIdx);
    } else {
      // Fallback: use face by variant so you can keep testing
      int v = MathUtil.clamp(caralho_var, 0, 5);
      base = selectReefFace(v);
      DriverStation.reportWarning(
          "[PoseTune/ALGAE] No/unknown tag (" + tag + "); fallback face=" + v, false);
    }

    Pose2d result =
        backoffAlongHeading(base, Math.abs(HighAltitudeConstants.ALGAE_RETRACT_OFFSET_M)); // always
                                                                                           // behind
    if (TELEMETRY) {
      SmartDashboard.putNumber("PoseTune/Tag", tag);
      SmartDashboard.putBoolean("PoseTune/Mapped", mapped);
      SmartDashboard.putNumber("PoseTune/ReefIdx", mapped ? reefIdx : -1);
      SmartDashboard.putString("PoseTune/BasePose", base.toString());
      SmartDashboard.putString("PoseTune/FinalPose", result.toString());
    }
    return result;
  }

  /**
   * Positive backM always offsets "behind" the target heading (away from its facing). This is
   * alliance/angle agnostic and prevents accidental forward offsets.
   */
  private static Pose2d backoffAlongHeading(Pose2d p, double backM) {
    if (backM <= 0)
      return p;
    double th = p.getRotation().getRadians();
    double x = p.getX() - backM * Math.cos(th);
    double y = p.getY() - backM * Math.sin(th);
    return new Pose2d(x, y, new Rotation2d(th));
  }
}
