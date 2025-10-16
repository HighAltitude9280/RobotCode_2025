package frc.robot.stateMachines;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.HighAltitudeConstantsPose;
import frc.robot.Robot;
import frc.robot.Robot.GameMode;

public final class TestTargets {
  private static volatile int variant = 0; // 0..3

  private TestTargets() {
  }

  public static void setVariant(int v) {
    variant = MathUtil.clamp(v, 0, 3);
  }

  public static int getVariant() {
    return variant;
  }

  public static Pose2d currentPose() {
    GameMode mode = Robot.getRobotContainer().getGameMode();
    switch (mode) {
      case CORAL_L1:
        return coralStationPose(variant);
      case CORAL_LX:
      case ALGAE:
        return reefBranchPose(variant);
      case MANUAL:
      default:
        return new Pose2d();
    }
  }

  // Usa NEAR/MIDDLE/FAR del lado actual (left/right) para probar intake de
  // estación
  private static Pose2d coralStationPose(int v) {
    int idx = MathUtil.clamp(v, 0, 2); // sólo 0..2
    boolean leftSide = Robot.isLeftMode();
    var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
    if (alliance == DriverStation.Alliance.Red) {
      return leftSide
          ? HighAltitudeConstantsPose.PATHFINDING_RED_LEFT_CORAL_STATION[idx]
          : HighAltitudeConstantsPose.PATHFINDING_RED_RIGHT_CORAL_STATION[idx];
    } else {
      return leftSide
          ? HighAltitudeConstantsPose.PATHFINDING_BLUE_LEFT_CORAL_STATION[idx]
          : HighAltitudeConstantsPose.PATHFINDING_BLUE_RIGHT_CORAL_STATION[idx];
    }
  }

  // Parea por letras: (A,B)=BC, (C,D)=BR, (E,F)=FR, (G,H)=FC; left/right elige la
  // letra
  private static Pose2d reefBranchPose(int v) {
    int pair = MathUtil.clamp(v, 0, 3); // 0:A/B, 1:C/D, 2:E/F, 3:G/H
    boolean leftBranch = Robot.isLeftMode(); // true=>A,C,E,G false=>B,D,F,H
    int idx = switch (pair) {
      case 0 -> leftBranch ? 0 : 1; // A/B
      case 1 -> leftBranch ? 2 : 3; // C/D
      case 2 -> leftBranch ? 4 : 5; // E/F
      case 3 -> leftBranch ? 6 : 7; // G/H
      default -> 0;
    };
    var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
    return (alliance == DriverStation.Alliance.Red)
        ? HighAltitudeConstantsPose.PATHFINDING_RED_BRANCHES[idx]
        : HighAltitudeConstantsPose.PATHFINDING_BLUE_BRANCHES[idx];
  }
}
