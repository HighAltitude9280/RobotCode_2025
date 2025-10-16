// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Central store for all field poses (reef centers, branches, feeders, coral
 * stations)
 * and helper enums related to pathfinding / targeting.
 *
 * All distances are in meters; headings are field-relative (Rotation2d).
 */
public final class HighAltitudeConstantsPose {

    private HighAltitudeConstantsPose() {
    } // constants holder

    // ---------------------------------------------------------------------------
    // Reef center poses (6 per alliance), ordered as REEF_POSITION enum: BC, BR,
    // FR, FC, FL, BL
    // ---------------------------------------------------------------------------

    public static final Pose2d[] PATHFINDING_BLUE_REEF_POS = {
            new Pose2d(2.963, 4.015, Rotation2d.fromDegrees(0)), // BC
            new Pose2d(3.704, 2.668, Rotation2d.fromDegrees(60)), // BR
            new Pose2d(5.282, 2.620, Rotation2d.fromDegrees(120)), // FR
            new Pose2d(5.975, 3.987, Rotation2d.fromDegrees(180)), // FC
            new Pose2d(5.253, 5.439, Rotation2d.fromDegrees(-120)), // FL
            new Pose2d(3.695, 5.439, Rotation2d.fromDegrees(-60)) // BL
    };

    public static final Pose2d[] PATHFINDING_RED_REEF_POS = {
            new Pose2d(14.587, 4.015, Rotation2d.fromDegrees(180)), // BC
            new Pose2d(13.855, 5.439, Rotation2d.fromDegrees(-120)), // BR
            new Pose2d(12.297, 5.439, Rotation2d.fromDegrees(-60)), // FR
            new Pose2d(11.500, 3.987, Rotation2d.fromDegrees(0)), // FC
            new Pose2d(12.268, 2.620, Rotation2d.fromDegrees(60)), // FL
            new Pose2d(13.846, 2.668, Rotation2d.fromDegrees(120)) // BL
    };

    /**
     * AprilTag IDs at the reef (order must match REEF_POSITIONS / REEF_POSITION).
     */
    public static final int[] BLUE_APRILTAG_IDS = { 18, 17, 22, 21, 20, 19 };
    public static final int[] RED_APRILTAG_IDS = { 7, 8, 9, 10, 11, 6 };

    // ---------------------------------------------------------------------------
    // Feeder poses
    // ---------------------------------------------------------------------------

    public static final Pose2d PATHFINDING_LEFT_BLUE_FEEDER = new Pose2d(1.160, 7.090, Rotation2d.fromDegrees(-54.2));
    public static final Pose2d PATHFINDING_RIGHT_BLUE_FEEDER = new Pose2d(1.130, 1.010, Rotation2d.fromDegrees(54.2));

    public static final Pose2d PATHFINDING_LEFT_RED_FEEDER = new Pose2d(16.410, 0.950, Rotation2d.fromDegrees(125.8));
    public static final Pose2d PATHFINDING_RIGHT_RED_FEEDER = new Pose2d(16.420, 7.060, Rotation2d.fromDegrees(-125.8));

    // ---------------------------------------------------------------------------
    // Coral station poses (left/right × far/middle/near)
    // ---------------------------------------------------------------------------

    public static final Pose2d[] PATHFINDING_BLUE_LEFT_CORAL_STATION = {
            new Pose2d(1.640, 7.440, Rotation2d.fromDegrees(-54.2)), // Left Far
            new Pose2d(1.160, 7.090, Rotation2d.fromDegrees(-54.2)), // Left Middle
            new Pose2d(0.620, 6.700, Rotation2d.fromDegrees(-54.2)) // Left Near
    };

    public static final Pose2d[] PATHFINDING_BLUE_RIGHT_CORAL_STATION = {
            new Pose2d(1.540, 0.660, Rotation2d.fromDegrees(54.2)), // Right Far
            new Pose2d(1.130, 1.010, Rotation2d.fromDegrees(54.2)), // Right Middle
            new Pose2d(0.650, 1.320, Rotation2d.fromDegrees(54.2)) // Right Near
    };

    public static final Pose2d[] PATHFINDING_RED_LEFT_CORAL_STATION = {
            new Pose2d(15.980, 0.630, Rotation2d.fromDegrees(125.8)), // Left Far
            new Pose2d(16.410, 0.950, Rotation2d.fromDegrees(125.8)), // Left Middle
            new Pose2d(16.930, 1.310, Rotation2d.fromDegrees(125.8)) // Left Near
    };

    public static final Pose2d[] PATHFINDING_RED_RIGHT_CORAL_STATION = {
            new Pose2d(15.920, 7.440, Rotation2d.fromDegrees(-125.8)), // Right Far
            new Pose2d(16.420, 7.060, Rotation2d.fromDegrees(-125.8)), // Right Middle
            new Pose2d(16.940, 6.700, Rotation2d.fromDegrees(-125.8)) // Right Near
    };

    // ---------------------------------------------------------------------------
    // Reef branches (12 per alliance), ordered A..L as in game manual
    // ---------------------------------------------------------------------------

    public static final Pose2d[] PATHFINDING_BLUE_BRANCHES = {
            new Pose2d(3.125, 4.180, Rotation2d.fromDegrees(0)), // A
            new Pose2d(3.125, 3.850, Rotation2d.fromDegrees(0)), // B
            new Pose2d(3.670, 2.950, Rotation2d.fromDegrees(60)), // C
            new Pose2d(3.960, 2.770, Rotation2d.fromDegrees(60)), // D
            new Pose2d(5.000, 2.770, Rotation2d.fromDegrees(120)), // E
            new Pose2d(5.290, 2.950, Rotation2d.fromDegrees(120)), // F
            new Pose2d(5.820, 3.850, Rotation2d.fromDegrees(180)), // G
            new Pose2d(5.820, 4.190, Rotation2d.fromDegrees(180)), // H
            new Pose2d(5.290, 5.100, Rotation2d.fromDegrees(-120)), // I
            new Pose2d(5.010, 5.260, Rotation2d.fromDegrees(-120)), // J
            new Pose2d(3.950, 5.260, Rotation2d.fromDegrees(-60)), // K
            new Pose2d(3.660, 5.100, Rotation2d.fromDegrees(-60)) // L
    };

    public static final Pose2d[] PATHFINDING_RED_BRANCHES = {
            new Pose2d(14.390, 3.850, Rotation2d.fromDegrees(180)), // A
            new Pose2d(14.390, 4.190, Rotation2d.fromDegrees(180)), // B
            new Pose2d(13.870, 5.090, Rotation2d.fromDegrees(-120)), // C
            new Pose2d(13.580, 5.260, Rotation2d.fromDegrees(-120)), // D
            new Pose2d(12.540, 5.260, Rotation2d.fromDegrees(-60)), // E
            new Pose2d(12.250, 5.200, Rotation2d.fromDegrees(-60)), // F
            new Pose2d(11.710, 4.190, Rotation2d.fromDegrees(0)), // G
            new Pose2d(11.710, 3.860, Rotation2d.fromDegrees(0)), // H
            new Pose2d(12.250, 2.940, Rotation2d.fromDegrees(60)), // I
            new Pose2d(12.530, 2.770, Rotation2d.fromDegrees(60)), // J
            new Pose2d(13.580, 2.780, Rotation2d.fromDegrees(120)), // K
            new Pose2d(13.860, 2.950, Rotation2d.fromDegrees(120)) // L
    };
    // --- Net & Processor approach poses (one per alliance) --- //TODO: llenarlas
    public static final Pose2d PATHFINDING_BLUE_NET = new Pose2d();
    public static final Pose2d PATHFINDING_RED_NET = new Pose2d();

    public static final Pose2d PATHFINDING_BLUE_PROCESSOR = new Pose2d();
    public static final Pose2d PATHFINDING_RED_PROCESSOR = new Pose2d();

    // ---------------------------------------------------------------------------
    // Algae removal: tag→level mapping and per-tag base poses (to be filled)
    // Order in POSES arrays MUST match the corresponding TAGS arrays.
    // ---------------------------------------------------------------------------

    public static final int[] BLUE_ALGAE_L3_TAGS = { 18, 22, 20 };
    public static final int[] BLUE_ALGAE_L2_TAGS = { 19, 17, 21 };
    public static final int[] RED_ALGAE_L3_TAGS = { 7, 9, 11 };
    public static final int[] RED_ALGAE_L2_TAGS = { 6, 8, 10 };

    public static final Pose2d[] BLUE_ALGAE_L3_POSES = {
            new Pose2d(), // Tag 18 TODO: fill
            new Pose2d(), // Tag 22
            new Pose2d() // Tag 20
    };

    public static final Pose2d[] BLUE_ALGAE_L2_POSES = {
            new Pose2d(), // Tag 19 TODO: fill
            new Pose2d(), // Tag 17
            new Pose2d() // Tag 21
    };

    public static final Pose2d[] RED_ALGAE_L3_POSES = {
            new Pose2d(), // Tag 7 TODO: fill
            new Pose2d(), // Tag 9
            new Pose2d() // Tag 11
    };

    public static final Pose2d[] RED_ALGAE_L2_POSES = {
            new Pose2d(), // Tag 6 TODO: fill
            new Pose2d(), // Tag 8
            new Pose2d() // Tag 10
    };

    // ---------------------------------------------------------------------------
    // Enums and helpers
    // ---------------------------------------------------------------------------

    public enum CORAL_STATION_POSITION {
        FAR(0), MIDDLE(1), NEAR(2);

        private final int id;

        CORAL_STATION_POSITION(int id) {
            this.id = id;
        }

        public int getID() {
            return id;
        }
    }

    /** Logical reef positions (6). */
    public enum REEF_POSITION {
        BC(0), BR(1), FR(2), FC(3), FL(4), BL(5);

        private final int id;

        REEF_POSITION(int id) {
            this.id = id;
        }

        public int getID() {
            return id;
        }

        /**
         * Maps a position (BC..BL) to branch index A..L using left/right:
         * left=true → 2*id, left=false → 2*id + 1
         */
        public int getBranchID(boolean left) {
            return left ? 2 * id : 2 * id + 1;
        }
    }

    /** Convenience array to keep order aligned with tag arrays and center poses. */
    public static final REEF_POSITION[] REEF_POSITIONS = {
            REEF_POSITION.BC, REEF_POSITION.BR, REEF_POSITION.FR,
            REEF_POSITION.FC, REEF_POSITION.FL, REEF_POSITION.BL
    };

    /**
     * Logical sides of the reef relative to the robot’s facing (back/front ring).
     * Use getPosition(frontMode) to resolve to a REEF_POSITION given current
     * front/back mode.
     */
    public enum REEF_SIDE {
        LEFT(REEF_POSITION.BL, REEF_POSITION.FL),
        CENTER(REEF_POSITION.BC, REEF_POSITION.FC),
        RIGHT(REEF_POSITION.BR, REEF_POSITION.FR);

        private final REEF_POSITION back;
        private final REEF_POSITION front;

        REEF_SIDE(REEF_POSITION back, REEF_POSITION front) {
            this.back = back;
            this.front = front;
        }

        /** Returns the position for the requested ring (front=true → front ring). */
        public REEF_POSITION getPosition(boolean front) {
            return front ? this.front : this.back;
        }
    }
}
