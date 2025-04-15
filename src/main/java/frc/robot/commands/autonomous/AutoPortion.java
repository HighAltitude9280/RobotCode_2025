// AutoPortion.java
package frc.robot.commands.autonomous;

import frc.robot.HighAltitudeConstants.REEF_HEIGHT;
import frc.robot.HighAltitudeConstants.REEF_POSITION;
import frc.robot.HighAltitudeConstants;
import frc.robot.HighAltitudeConstants.CORAL_STATION_POSITION; // Enum for Coral Station (FAR, MIDDLE, NEAR)
import edu.wpi.first.math.geometry.Pose2d;

public class AutoPortion {
    private REEF_POSITION pos;

    public REEF_POSITION getPos() {
        return pos;
    }

    private boolean leftBranch;

    public boolean isLeftBranch() {
        return leftBranch;
    }

    private REEF_HEIGHT height;

    public REEF_HEIGHT getHeight() {
        return height;
    }

    private Boolean leftFeeder;

    public Boolean isLeftFeeder() {
        return leftFeeder;
    }

    // New field for Coral Station position (FAR, MIDDLE, NEAR)
    private CORAL_STATION_POSITION coralStationPos;

    public CORAL_STATION_POSITION getCoralStationPos() {
        return coralStationPos;
    }

    /**
     * Constructor for an autonomous portion.
     * 
     * @param pos             The REEF_POSITION for reef portions (can be null if
     *                        not applicable).
     * @param leftBranch      True for left branch, false for right branch.
     * @param height          The target REEF_HEIGHT.
     * @param leftFeeder      True for left feeder, false for right feeder (can be
     *                        null).
     * @param coralStationPos The CORAL_STATION_POSITION for coral station portions;
     *                        null for reef portions.
     */
    public AutoPortion(REEF_POSITION pos, boolean leftBranch, REEF_HEIGHT height, Boolean leftFeeder,
            CORAL_STATION_POSITION coralStationPos) {
        this.pos = pos;
        this.leftBranch = leftBranch;
        this.leftFeeder = leftFeeder;
        this.height = height;
        this.coralStationPos = coralStationPos;
    }

    /**
     * Computes the approach pose for the given final target pose using an offset.
     * If intakeMode is false (normal mode for REEF), the offset is subtracted
     * (approach from behind).
     * If intakeMode is true (for Coral Station or feeder intake), the offset is
     * added (approach from the front).
     * For horizontal branches (angles near 0° or 180°), only the X component is
     * offset (with inverted behavior for intake mode).
     *
     * @param finalPose  The final target pose.
     * @param intakeMode True to calculate the approach pose for intake (Coral
     *                   Station), false for normal REEF.
     * @return The computed approach pose.
     */
    public Pose2d getApproachPose(Pose2d finalPose, boolean intakeMode) {
        double offsetDistance = HighAltitudeConstants.PATHFINDING_APPROACH_OFFSET; // e.g., 0.9 m
        double angleDeg = finalPose.getRotation().getDegrees();
        double newX, newY;

        // For horizontal branches: angles near 0° or 180° (or -180°)
        if (Math.abs(angleDeg) < 5 || Math.abs(Math.abs(angleDeg) - 180) < 5) {
            if (Math.abs(angleDeg) < 5) {
                // Normal mode: subtract offset in X; intake mode: add offset in X.
                newX = intakeMode ? finalPose.getX() + offsetDistance : finalPose.getX() - offsetDistance;
            } else { // angle near 180°
                // Normal mode: add offset; intake mode: subtract offset.
                newX = intakeMode ? finalPose.getX() - offsetDistance : finalPose.getX() + offsetDistance;
            }
            newY = finalPose.getY();
        } else {
            // For non-horizontal branches, use generic calculation.
            double angleRad = finalPose.getRotation().getRadians();
            if (intakeMode) {
                newX = finalPose.getX() + offsetDistance * Math.cos(angleRad);
                newY = finalPose.getY() + offsetDistance * Math.sin(angleRad);
            } else {
                newX = finalPose.getX() - offsetDistance * Math.cos(angleRad);
                newY = finalPose.getY() - offsetDistance * Math.sin(angleRad);
            }
        }
        return new Pose2d(newX, newY, finalPose.getRotation());
    }
}
