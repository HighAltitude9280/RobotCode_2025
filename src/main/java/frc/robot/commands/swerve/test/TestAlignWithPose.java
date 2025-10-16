package frc.robot.commands.swerve.test;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.SwerveDriveTrain;

public class TestAlignWithPose extends Command {
  private final SwerveDriveTrain swerve;
  private final Supplier<Pose2d> targetSupplier;
  private final double vMax, wMax;

  private boolean onTarget = false;
  private int stableCount = 0;
  private static final int STABLE_CYCLES = 5; // 5 * 20ms ≈ 100ms estable

  public TestAlignWithPose(Supplier<Pose2d> targetSupplier,
      double vMax, double wMax) {
    this.swerve = Robot.getRobotContainer().getSwerveDriveTrain();
    this.targetSupplier = targetSupplier;
    this.vMax = vMax;
    this.wMax = wMax;
    addRequirements(swerve);
  }

  @Override
  public void execute() {
    onTarget = swerve.AlignWithTargetPose(
        targetSupplier.get(), vMax, wMax);
    stableCount = onTarget ? (stableCount + 1) : 0;
  }

  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();
  }

  @Override
  public boolean isFinished() {
    return stableCount >= STABLE_CYCLES;
  }
}
