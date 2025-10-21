// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Central store for all field poses (reef centers, branches, feeders, coral stations) and helper
 * enums related to pathfinding / targeting.
 *
 * All distances are in meters; headings are field-relative (Rotation2d).
 */
public final class HighAltitudeConstantsPose {

        private HighAltitudeConstantsPose() {} // constants holder

        // ---------------------------------------------------------------------------
        // Reef center poses (6 per alliance), ordered as REEF_POSITION enum: BC, BR,
        // FR, FC, FL, BL
        // ---------------------------------------------------------------------------

        public static final Pose2d[] PATHFINDING_BLUE_REEF_POS =
                        {new Pose2d(2.963, 4.015, Rotation2d.fromDegrees(0)), // BC
                                        new Pose2d(3.704, 2.668, Rotation2d.fromDegrees(60)), // BR
                                        new Pose2d(5.282, 2.620, Rotation2d.fromDegrees(120)), // FR
                                                                                               // bien
                                        new Pose2d(5.975, 3.987, Rotation2d.fromDegrees(180)), // FC
                                        new Pose2d(5.253, 5.439, Rotation2d.fromDegrees(-120)), // FL
                                                                                                // bien
                                        new Pose2d(3.695, 5.439, Rotation2d.fromDegrees(-60)) // BL
                        };

        public static final Pose2d[] PATHFINDING_RED_REEF_POS =
                        {new Pose2d(14.587, 4.015, Rotation2d.fromDegrees(180)), // BC
                                        new Pose2d(13.855, 5.439, Rotation2d.fromDegrees(-120)), // BR
                                        new Pose2d(12.297, 5.439, Rotation2d.fromDegrees(-60)), // FR
                                                                                                // bien
                                        new Pose2d(11.500, 3.987, Rotation2d.fromDegrees(0)), // FC
                                        new Pose2d(12.268, 2.620, Rotation2d.fromDegrees(60)), // FL
                                        new Pose2d(13.846, 2.668, Rotation2d.fromDegrees(120)) // BL
                        };

        /**
         * AprilTag IDs at the reef (order must match REEF_POSITIONS / REEF_POSITION).
         */
        public static final int[] BLUE_APRILTAG_IDS = {18, 17, 22, 21, 20, 19};
        public static final int[] RED_APRILTAG_IDS = {7, 8, 9, 10, 11, 6};

        // ---------------------------------------------------------------------------
        // Feeder poses
        // ---------------------------------------------------------------------------

        public static final Pose2d PATHFINDING_LEFT_BLUE_FEEDER =
                        new Pose2d(1.160, 7.090, Rotation2d.fromDegrees(-54.2));
        public static final Pose2d PATHFINDING_RIGHT_BLUE_FEEDER =
                        new Pose2d(1.130, 1.010, Rotation2d.fromDegrees(54.2));

        public static final Pose2d PATHFINDING_LEFT_RED_FEEDER =
                        new Pose2d(16.410, 0.950, Rotation2d.fromDegrees(125.8));
        public static final Pose2d PATHFINDING_RIGHT_RED_FEEDER =
                        new Pose2d(16.420, 7.060, Rotation2d.fromDegrees(-125.8));

        // ---------------------------------------------------------------------------
        // Coral station poses (left/right × far/middle/near)
        // ---------------------------------------------------------------------------

        public static final Pose2d[] PATHFINDING_BLUE_LEFT_CORAL_STATION =
                        {new Pose2d(1.640, 7.440, Rotation2d.fromDegrees(-54.2)), // Left Far
                                        new Pose2d(1.235, 7.364, Rotation2d.fromDegrees(-55)), // Left
                                                                                               // Middle
                                        new Pose2d(0.620, 6.700, Rotation2d.fromDegrees(-54.2)) // Left
                                                                                                // Near
                        };

        public static final Pose2d[] PATHFINDING_BLUE_RIGHT_CORAL_STATION =
                        {new Pose2d(1.540, 0.660, Rotation2d.fromDegrees(54.2)), // Right Far
                                        new Pose2d(1.367, 0.603, Rotation2d.fromDegrees(55)), // Right
                                                                                              // Middle
                                        new Pose2d(0.650, 1.320, Rotation2d.fromDegrees(54.2)) // Right
                                                                                               // Near
                        };

        public static final Pose2d[] PATHFINDING_RED_LEFT_CORAL_STATION =
                        {new Pose2d(15.980, 0.630, Rotation2d.fromDegrees(125.8)), // Left Far
                                        new Pose2d(16.183, 0.686, Rotation2d.fromDegrees(125)), // Left
                                                                                                // Middle
                                        new Pose2d(16.930, 1.310, Rotation2d.fromDegrees(125.8)) // Left
                                                                                                 // Near
                        };

        public static final Pose2d[] PATHFINDING_RED_RIGHT_CORAL_STATION =
                        {new Pose2d(15.920, 7.440, Rotation2d.fromDegrees(-125.8)), // Right Far
                                        new Pose2d(16.076, 7.424, Rotation2d.fromDegrees(-125)), // Right
                                                                                                 // Middle
                                        new Pose2d(16.940, 6.700, Rotation2d.fromDegrees(-125.8)) // Right
                                                                                                  // Near
                        };

        // ---------------------------------------------------------------------------
        // Reef branches (12 per alliance), ordered A..L as in game manual
        // ---------------------------------------------------------------------------

        public static final Pose2d[] PATHFINDING_BLUE_BRANCHES =
                        {new Pose2d(3.239, 4.205, Rotation2d.fromDegrees(-4.459)), // A
                                        new Pose2d(3.209, 3.855, Rotation2d.fromDegrees(-1.754)), // B
                                        new Pose2d(3.695, 3.040, Rotation2d.fromDegrees(54.403)), // C
                                        new Pose2d(3.970, 2.843, Rotation2d.fromDegrees(56.743)), // D
                                        new Pose2d(4.961, 2.855, Rotation2d.fromDegrees(114.523)), // E
                                        new Pose2d(5.260, 2.997, Rotation2d.fromDegrees(118.430)), // F
                                        new Pose2d(5.752, 3.901, Rotation2d.fromDegrees(178.655)), // G
                                        new Pose2d(5.757, 4.173, Rotation2d.fromDegrees(179.371)), // H
                                        new Pose2d(5.278, 5.019, Rotation2d.fromDegrees(-125.049)), // I
                                        new Pose2d(5.020, 5.199, Rotation2d.fromDegrees(-121.780)), // J
                                        new Pose2d(4.016, 5.284, Rotation2d.fromDegrees(-62.888)), // K
                                        new Pose2d(3.724, 5.061, Rotation2d.fromDegrees(-62.788)) // L
                        };

        public static final Pose2d[] PATHFINDING_RED_BRANCHES =
                        {new Pose2d(14.310, 3.873, Rotation2d.fromDegrees(177.199)), // A
                                        new Pose2d(14.334, 4.178, Rotation2d.fromDegrees(178.577)), // B
                                        new Pose2d(13.816, 5.037, Rotation2d.fromDegrees(-123.300)), // C
                                        new Pose2d(13.544, 5.231, Rotation2d.fromDegrees(-120.013)), // D
                                        new Pose2d(12.608, 5.212, Rotation2d.fromDegrees(-63.615)), // E
                                        new Pose2d(12.315, 5.071, Rotation2d.fromDegrees(-62.207)), // F
                                        new Pose2d(11.808, 4.205, Rotation2d.fromDegrees(-3.318)), // G
                                        new Pose2d(11.784, 3.875, Rotation2d.fromDegrees(-1.090)), // H
                                        new Pose2d(12.265, 3.041, Rotation2d.fromDegrees(56.268)), // I
                                        new Pose2d(12.575, 2.835, Rotation2d.fromDegrees(59.627)), // J
                                        new Pose2d(13.540, 2.851, Rotation2d.fromDegrees(116.721)), // K
                                        new Pose2d(13.876, 3.019, Rotation2d.fromDegrees(120.057)) // L
                        };

        // --- Net & Processor approach poses (one per alliance) --- //TODO: tunearlos
        public static final Pose2d PATHFINDING_BLUE_NET =
                        new Pose2d(7.700, 5.000, Rotation2d.fromDegrees(0));
        public static final Pose2d PATHFINDING_RED_NET =
                        new Pose2d(9.800, 3.000, Rotation2d.fromDegrees(180));

        public static final Pose2d PATHFINDING_BLUE_PROCESSOR =
                        new Pose2d(6.000, 0.570, Rotation2d.fromDegrees(-90));
        public static final Pose2d PATHFINDING_RED_PROCESSOR =
                        new Pose2d(11.500, 7.500, Rotation2d.fromDegrees(90));

        // ---------------------------------------------------------------------------
        // Algae removal: tag -> level mapping and per-tag base poses
        // Order in POSES arrays MUST match the corresponding TAGS arrays.
        // ---------------------------------------------------------------------------

        public static final int[] BLUE_ALGAE_L3_TAGS = {18, 22, 20};// FC, BR & BL
        public static final int[] BLUE_ALGAE_L2_TAGS = {19, 17, 21}; // FL, FR & BC
        public static final int[] RED_ALGAE_L3_TAGS = {7, 9, 11}; // FC, BR & BL
        public static final int[] RED_ALGAE_L2_TAGS = {6, 8, 10}; // FL, FR & BC

        // TODO: tunear
        public static final Pose2d[] BLUE_ALGAE_L3_POSES =
                        {new Pose2d(3.180, 4.000, Rotation2d.fromDegrees(0)),
                                        // Tag 18,BC

                                        new Pose2d(5.167, 2.868, Rotation2d.fromDegrees(120)),
                                        // Tag 22,FR

                                        new Pose2d(5.143, 5.170, Rotation2d.fromDegrees(-120))
                        // Tag 20,FL
                        };

        public static final Pose2d[] BLUE_ALGAE_L2_POSES =
                        {new Pose2d(3.860, 5.170, Rotation2d.fromDegrees(-60)),
                                        // Tag 19, BL

                                        new Pose2d(3.836, 2.868, Rotation2d.fromDegrees(60)),
                                        // Tag 17, BR

                                        new Pose2d(5.790, 4.000, Rotation2d.fromDegrees(180))
                        // Tag 21, FC
                        };

        public static final Pose2d[] RED_ALGAE_L3_POSES =
                        {new Pose2d(14.373, 4.019, Rotation2d.fromDegrees(180)),
                                        // Tag 7, BC

                                        new Pose2d(12.407, 5.134, Rotation2d.fromDegrees(-60)),
                                        // Tag 9, FR

                                        new Pose2d(12.383, 2.868, Rotation2d.fromDegrees(60))
                        // Tag 11, FL
                        };

        public static final Pose2d[] RED_ALGAE_L2_POSES =
                        {new Pose2d(13.738, 2.868, Rotation2d.fromDegrees(120)),
                                        // Tag 6, BL

                                        new Pose2d(13.714, 5.134, Rotation2d.fromDegrees(-120)),
                                        // Tag 8, BR

                                        new Pose2d(11.760, 4.019, Rotation2d.fromDegrees(0))
                        // Tag 10, FC
                        };

        public static final Pose2d CORAL_STATION_L1_BLUE_LEFT =
                        new Pose2d(1.103, 6.956, Rotation2d.fromDegrees(125.0));
        public static final Pose2d CORAL_STATION_L1_BLUE_RIGHT =
                        new Pose2d(1.103, 1.058, Rotation2d.fromDegrees(-125.0));

        public static final Pose2d CORAL_STATION_L1_RED_LEFT =
                        new Pose2d(16.076, 0.782, Rotation2d.fromDegrees(-55.0));
        public static final Pose2d CORAL_STATION_L1_RED_RIGHT =
                        new Pose2d(16.076, 7.256, Rotation2d.fromDegrees(55.0));

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
                 * Maps a position (BC..BL) to branch index A..L using left/right: left=true → 2*id,
                 * left=false → 2*id + 1
                 */
                public int getBranchID(boolean left) {
                        return left ? 2 * id : 2 * id + 1;
                }
        }

        /** Convenience array to keep order aligned with tag arrays and center poses. */
        public static final REEF_POSITION[] REEF_POSITIONS = {REEF_POSITION.BC, REEF_POSITION.BR,
                        REEF_POSITION.FR, REEF_POSITION.FC, REEF_POSITION.FL, REEF_POSITION.BL};

        /**
         * Logical sides of the reef relative to the robot’s facing (back/front ring). Use
         * getPosition(frontMode) to resolve to a REEF_POSITION given current front/back mode.
         */
        public enum REEF_SIDE {
                LEFT(REEF_POSITION.BL, REEF_POSITION.FL), CENTER(REEF_POSITION.BC,
                                REEF_POSITION.FC), RIGHT(REEF_POSITION.BR, REEF_POSITION.FR);

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
