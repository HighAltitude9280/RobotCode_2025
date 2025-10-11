package frc.robot.subsystems.vision;

import java.util.Optional;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;

public final class LastPoseCache {
    private Pose2d last;
    private double ts;

    /**
     * Updates the cached {@link Pose2d} and records the current timestamp.
     * <p>
     * If the provided pose is non-null, this method stores it as the last known
     * pose
     * and updates the internal timestamp to the current FPGA time.
     * If {@code pose} is {@code null}, the cache and timestamp remain unchanged.
     *
     * @param pose the new {@link Pose2d} to store; ignored if {@code null}
     */

    public void update(Pose2d pose) {
        if (pose != null) {
            last = pose;
            ts = Timer.getFPGATimestamp();
        }
    }

    /**
     * Returns the most recently stored {@link Pose2d}, regardless of its age.
     * <p>
     * This method does not perform any freshness checks or timestamp validation.
     * It simply returns the last pose that was stored, which may be {@code null}
     * if no pose has been set yet.
     *
     * @return the last cached {@link Pose2d}, or {@code null} if none has been set
     */

    public Pose2d getLastPose() {
        return last;
    }

    /**
     * Sets the last cached {@link Pose2d} to the given pose value.
     * <p>
     * This method updates the internal reference without modifying the timestamp.
     * It is typically used when the timestamp is managed externally or when
     * only the stored pose needs to be replaced.
     *
     * @param pose the {@link Pose2d} to store as the last known pose
     */

    public void setLastPose(Pose2d pose) {
        last = pose;
    }

    /**
     * Returns the most recently stored {@link Pose2d} if it is still considered
     * "fresh".
     * <p>
     * A pose is considered fresh if the time elapsed since it was last updated
     * does not exceed the specified maximum age in seconds. If the cached pose
     * is too old or has never been set, this method returns an empty
     * {@link Optional}.
     *
     * @param maxAgeSec the maximum allowed age (in seconds) for the cached pose to
     *                  be considered valid
     * @return an {@link Optional} containing the last cached {@link Pose2d} if it
     *         is fresh;
     *         otherwise, {@link Optional#empty()}
     */

    public Optional<Pose2d> getIfFresh(double maxAgeSec) {
        if (last == null) {
            return Optional.empty();
        }

        double age = Timer.getFPGATimestamp() - ts;
        return (age <= maxAgeSec) ? Optional.of(last) : Optional.empty();
    }

    /**
     * Returns the age of the last cached {@link Pose2d} in seconds.
     * <p>
     * The age is calculated as the time elapsed since the last update.
     * If no pose has been stored yet, this method returns
     * {@link Double#POSITIVE_INFINITY}.
     *
     * @return the age of the last cached pose in seconds, or
     *         {@link Double#POSITIVE_INFINITY} if none has been set
     */

    public double ageSec() {
        return (last == null) ? Double.POSITIVE_INFINITY : Timer.getFPGATimestamp() - ts;
    }
}
