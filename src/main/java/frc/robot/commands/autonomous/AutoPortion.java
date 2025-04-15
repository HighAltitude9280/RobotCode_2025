// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
     * For horizontal branches (angles near 0° or 180°), only the X component is
     * offset.
     * For all other branches, a generic calculation is applied.
     *
     * @param finalPose The final target pose.
     * @return The computed approach pose.
     */
    public Pose2d getApproachPose(Pose2d finalPose) {
        // Get the offset distance from HighAltitudeConstants (tunable during
        // competition)
        double offsetDistance = HighAltitudeConstants.PATHFINDING_APPROACH_OFFSET;
        double angleDeg = finalPose.getRotation().getDegrees();
        double newX, newY;

        // For horizontal branches: if angle is within 5° of 0° or 180° (or -180°)
        if (Math.abs(angleDeg) < 5 || Math.abs(Math.abs(angleDeg) - 180) < 5) {
            // For angle near 0°, assume target faces positive X; subtract offset to
            // approach from behind
            if (Math.abs(angleDeg) < 5) {
                newX = finalPose.getX() - offsetDistance;
            } else { // angle near 180°: facing negative X; add offset to approach from behind
                newX = finalPose.getX() + offsetDistance;
            }
            newY = finalPose.getY();
        } else {
            // Generic approach: apply offset in both X and Y based on the target's
            // orientation.
            double angleRad = finalPose.getRotation().getRadians();
            newX = finalPose.getX() - offsetDistance * Math.cos(angleRad);
            newY = finalPose.getY() - offsetDistance * Math.sin(angleRad);
        }
        return new Pose2d(newX, newY, finalPose.getRotation());
    }
}
