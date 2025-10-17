// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.HighAltitudeConstants.PoseIdx;
import frc.robot.HighAltitudeConstants.REEF_HEIGHT;
import frc.robot.HighAltitudeConstantsPose.REEF_SIDE;
import frc.robot.Robot.GameMode;
import frc.robot.commands.cancel.PathCancelCommand;
import frc.robot.commands.cancel.ResetLiftEncoders;
import frc.robot.commands.extensor.compound.both.CoralOrAlgaeLiftDown;
import frc.robot.commands.extensor.compound.both.LiftWristGoToPose;
import frc.robot.commands.extensor.compound.both.LiftWristGoToTargetHeight;
import frc.robot.commands.extensor.gripper.IntakeAuto;
import frc.robot.commands.extensor.gripper.algae.IntakeAlgaeAuto;
import frc.robot.commands.extensor.gripper.manual.IntakeAlgae;
import frc.robot.commands.extensor.gripper.manual.ScoreGamePiece;
import frc.robot.commands.extensor.lift.manual.LiftDown;
import frc.robot.commands.extensor.lift.manual.LiftUp;
import frc.robot.commands.extensor.wrist.control.WristGoToTarget;
import frc.robot.commands.modes.SetCoralMode;
import frc.robot.commands.modes.SetFrontMode;
import frc.robot.commands.modes.SetLeftMode;
import frc.robot.commands.modes.SetReefSideMode;
import frc.robot.commands.modes.ToggleCoralMode;
import frc.robot.commands.modes.TogglePrecisionMode;
import frc.robot.commands.modes.WhileHeldPrecisionMode;
import frc.robot.commands.oneDriver.AlignWithBranchAndScore;
import frc.robot.commands.swerve.autonomous.SwerveMoveMeters;
import frc.robot.commands.swerve.autonomous.TurnWheels;
import frc.robot.commands.swerve.autonomous.offSeason.AlignWithTargetPose;
import frc.robot.commands.swerve.autonomous.offSeason.DriveToCoralStation;
import frc.robot.commands.swerve.swerveParameters.ResetOdometryZeros;
import frc.robot.commands.swerve.swerveParameters.SetIsFieldOriented;
import frc.robot.resources.joysticks.HighAltitudeJoystick;
import frc.robot.resources.joysticks.HighAltitudeJoystick.AxisType;
import frc.robot.resources.joysticks.HighAltitudeJoystick.ButtonType;
import frc.robot.resources.joysticks.HighAltitudeJoystick.JoystickType;
import frc.robot.stateMachines.NextModeCommand;
import frc.robot.stateMachines.OIHelpers;
import frc.robot.stateMachines.OIHelpers.LiftWristGoToSelectedPose;
import frc.robot.stateMachines.OIHelpers.SetSelectedPoseIdxCommand;
import frc.robot.stateMachines.ToggleManualCommand;

/** Add your docs here. */
public class OI {
        public static OI instance;

        private HighAltitudeJoystick pilot;
        private HighAltitudeJoystick copilot;

        public void ConfigureButtonBindings() {
                ////////////////////////// PILOT //////////////////////////

                switch (HighAltitudeConstants.CURRENT_PILOT) {

                        case Joakin:

                                pilot = new HighAltitudeJoystick(0, JoystickType.XBOX);

                                pilot.setAxisDeadzone(AxisType.LEFT_X, 0.1);
                                pilot.setAxisDeadzone(AxisType.LEFT_Y, 0.1);
                                pilot.setAxisDeadzone(AxisType.RIGHT_X, 0.1);

                                pilot.onTrue(ButtonType.BACK, new SetIsFieldOriented(true));
                                pilot.onTrue(ButtonType.START, new SetIsFieldOriented(false));
                                pilot.onTrueCombo(new ResetOdometryZeros(), ButtonType.START,
                                                ButtonType.BACK);

                                pilot.whileTrue(ButtonType.POV_E, new SetLeftMode(false));

                                pilot.whileTrue(ButtonType.POV_W, new SetLeftMode(true));
                                /*
                                 * pilot.whileTrue(ButtonType.POV_W, new
                                 * PathplanToReefThenVisionPose(null, null, true,
                                 * HighAltitudeConstants.VISION_POSE_MAX_SPEED,
                                 * HighAltitudeConstants.VISION_POSE_MAX_TURN)); *
                                 */

                                pilot.whileTrue(ButtonType.POV_N, new SetFrontMode(true));
                                pilot.whileTrue(ButtonType.POV_S, new SetFrontMode(false));

                                pilot.whileTrue(ButtonType.X, new SetReefSideMode(REEF_SIDE.LEFT));
                                pilot.whileTrue(ButtonType.A,
                                                new SetReefSideMode(REEF_SIDE.CENTER));
                                pilot.whileTrue(ButtonType.B, new SetReefSideMode(REEF_SIDE.RIGHT));

                                pilot.whileTrue(ButtonType.Y, new WhileHeldPrecisionMode());

                                pilot.whileTrue(ButtonType.LB, new AlignWithTargetPose(null, null,
                                                true, HighAltitudeConstants.VISION_POSE_MAX_SPEED,
                                                HighAltitudeConstants.VISION_POSE_MAX_TURN));
                                pilot.onFalse(ButtonType.LB, new SequentialCommandGroup(
                                                new TurnWheels(0).withTimeout(0.5),
                                                new SwerveMoveMeters(0.2, 0,
                                                                HighAltitudeConstants.VISION_POSE_MAX_SPEED)
                                                                                .withTimeout(0.25)));

                                // pilot.whileTrue(ButtonType.LB, new AlignVisionMoveMeters(true));

                                // pilot.whileTrue(ButtonType.RB, new AlignVisionMoveMeters(false));

                                pilot.whileTrue(ButtonType.RB, new AlignWithTargetPose(null, null,
                                                false, HighAltitudeConstants.VISION_POSE_MAX_SPEED,
                                                HighAltitudeConstants.VISION_POSE_MAX_TURN));
                                pilot.onFalse(ButtonType.RB, new SequentialCommandGroup(
                                                new TurnWheels(0).withTimeout(0.5),
                                                new SwerveMoveMeters(0.2, 0,
                                                                HighAltitudeConstants.VISION_POSE_MAX_SPEED)
                                                                                .withTimeout(0.25)));

                                pilot.onTrue(ButtonType.LT, new CoralOrAlgaeLiftDown());
                                pilot.whileTrue(ButtonType.RT, new DriveToCoralStation(false, null,
                                                false, HighAltitudeConstants.VISION_POSE_MAX_SPEED,
                                                HighAltitudeConstants.VISION_POSE_MAX_TURN));

                                break;
                        case OneDriver:

                                pilot = new HighAltitudeJoystick(0, JoystickType.XBOX);

                                pilot.setAxisDeadzone(AxisType.LEFT_X, 0.1);
                                pilot.setAxisDeadzone(AxisType.LEFT_Y, 0.1);
                                pilot.setAxisDeadzone(AxisType.RIGHT_X, 0.1);

                                pilot.onTrue(ButtonType.BACK, new SetIsFieldOriented(true));
                                pilot.onTrue(ButtonType.START, new SetIsFieldOriented(false));
                                pilot.onTrueCombo(new ResetOdometryZeros(), ButtonType.START,
                                                ButtonType.BACK);

                                pilot.whileTrue(ButtonType.Y, new WhileHeldPrecisionMode()); // binded
                                                                                             // to a
                                                                                             // paddle

                                pilot.onTrue(ButtonType.LB,
                                                new AlignWithBranchAndScore(true, REEF_HEIGHT.TOP));
                                pilot.onTrue(ButtonType.LT,
                                                new AlignWithBranchAndScore(true, REEF_HEIGHT.L3));
                                pilot.onTrue(ButtonType.POV_W,
                                                new AlignWithBranchAndScore(true, REEF_HEIGHT.L2));

                                pilot.onTrue(ButtonType.RB, new AlignWithBranchAndScore(false,
                                                REEF_HEIGHT.TOP));
                                pilot.onTrue(ButtonType.RT,
                                                new AlignWithBranchAndScore(false, REEF_HEIGHT.L3));
                                pilot.onTrue(ButtonType.POV_E,
                                                new AlignWithBranchAndScore(false, REEF_HEIGHT.L2));


                                pilot.whileTrue(ButtonType.A,
                                                new LiftWristGoToTargetHeight(REEF_HEIGHT.BOTTOM));

                                pilot.whileTrue(ButtonType.B, new DriveToCoralStation(false, false,
                                                HighAltitudeConstants.VISION_POSE_MAX_SPEED,
                                                HighAltitudeConstants.VISION_POSE_MAX_TURN));

                                pilot.whileTrue(ButtonType.POV_S, new IntakeAuto());
                                pilot.whileTrue(ButtonType.X, new ToggleCoralMode());

                                pilot.whileTrueCombo(new PathCancelCommand(), ButtonType.LS,
                                                ButtonType.RS);

                                pilot.whileTrue(ButtonType.LS, new ScoreGamePiece(-0.1));
                                pilot.whileTrue(ButtonType.RS, new ScoreGamePiece(0.1));

                                break;

                        case OnlySwerve:



                                pilot = new HighAltitudeJoystick(0, JoystickType.XBOX);

                                java.util.function.Supplier<GameMode> mode =
                                                () -> Robot.getRobotContainer().getGameMode();



                                // ==== Cambio de modo & Toggle Manual ====
                                // pilot.onTrue(ButtonType.START, new NextModeCommand());

                                // pilot.onTrue(ButtonType.BACK, new NextModeCommand());
                                pilot.setAxisDeadzone(AxisType.LEFT_X, 0.1);
                                pilot.setAxisDeadzone(AxisType.LEFT_Y, 0.1);
                                pilot.setAxisDeadzone(AxisType.RIGHT_X, 0.1);

                                /*
                                 * pilot.onTrue(ButtonType.BACK, new SetIsFieldOriented(true));
                                 * pilot.onTrue(ButtonType.START, new SetIsFieldOriented(false));
                                 * pilot.onTrueCombo(new ResetOdometryZeros(), ButtonType.START,
                                 * ButtonType.BACK);
                                 */

                                pilot.whileTrue(ButtonType.POV_W, new WhileHeldPrecisionMode());
                                pilot.whileTrue(ButtonType.X, new TogglePrecisionMode());


                                pilot.onTrue(ButtonType.LB, OIHelpers.onlyInMode(mode,
                                                GameMode.CORAL_L1,
                                                new AlignWithTargetPose(true,
                                                                HighAltitudeConstants.VISION_POSE_MAX_SPEED,
                                                                HighAltitudeConstants.VISION_POSE_MAX_TURN,
                                                                true)));

                                pilot.whileTrue(ButtonType.LB, OIHelpers.onlyInMode(mode,
                                                GameMode.CORAL_L1,
                                                new InstantCommand(() -> System.out.println(
                                                                "LB hace desde estado: " + Robot
                                                                                .getRobotContainer()
                                                                                .getGameMode()))));

                                pilot.onTrue(ButtonType.LB, OIHelpers.onlyInMode(mode,
                                                GameMode.CORAL_LX,
                                                new AlignWithBranchAndScore(true, REEF_HEIGHT.L3)));

                                pilot.whileTrue(ButtonType.LB, OIHelpers.onlyInMode(mode,
                                                GameMode.CORAL_LX,
                                                new InstantCommand(() -> System.out.println(
                                                                "LB hace desde estado: " + Robot
                                                                                .getRobotContainer()
                                                                                .getGameMode()))));


                                pilot.onTrue(ButtonType.RB, new AlignWithTargetPose(false,
                                                HighAltitudeConstants.VISION_POSE_MAX_SPEED,
                                                HighAltitudeConstants.VISION_POSE_MAX_TURN, true));

                                /*
                                 * pilot.whileTrue(ButtonType.RB, OIHelpers.onlyInMode(mode,
                                 * GameMode.CORAL_L1, new InstantCommand(() -> System.out.println(
                                 * "LB hace desde estado: " + Robot .getRobotContainer()
                                 * .getGameMode()))));
                                 */
                                break;

                        case JoakinButChambing:

                                pilot = new HighAltitudeJoystick(0, JoystickType.XBOX);

                                pilot.setAxisDeadzone(AxisType.LEFT_X, 0.1);
                                pilot.setAxisDeadzone(AxisType.LEFT_Y, 0.1);
                                pilot.setAxisDeadzone(AxisType.RIGHT_X, 0.1);

                                // pilot.whileTrue(ButtonType.Y, new WhileHeldPrecisionMode());

                                pilot.onTrue(ButtonType.BACK, new SetIsFieldOriented(true));
                                pilot.onTrue(ButtonType.START, new SetIsFieldOriented(false));
                                pilot.onTrueCombo(new ResetOdometryZeros(), ButtonType.START,
                                                ButtonType.BACK);

                                pilot.whileTrue(ButtonType.LB, new IntakeAuto());

                                pilot.whileTrue(ButtonType.RT, new ScoreGamePiece(
                                                HighAltitudeConstants.GRIPPER_IN_SPEED));

                                pilot.whileTrue(ButtonType.LT, new IntakeAlgae());

                                pilot.whileTrue(ButtonType.X, new PathCancelCommand());

                                // pilot.whileTrue(ButtonType.A, new WristSetAngleTarget(42));
                                // pilot.whileTrue(ButtonType.B, new WristSetAngleTarget(82));
                                // pilot.whileTrue(ButtonType.Y, new WristSetAngleTarget(0));

                                pilot.onTrue(ButtonType.POV_N,
                                                new LiftWristGoToTargetHeight(REEF_HEIGHT.BOTTOM));

                                pilot.onTrue(ButtonType.POV_E,
                                                new LiftWristGoToTargetHeight(REEF_HEIGHT.L2));

                                pilot.onTrue(ButtonType.POV_S,
                                                new LiftWristGoToTargetHeight(REEF_HEIGHT.L3));

                                pilot.onTrue(ButtonType.POV_W,
                                                new LiftWristGoToTargetHeight(REEF_HEIGHT.TOP));

                                /*
                                 * pilot.whileTrue(ButtonType.POV_N,
                                 * Robot.getRobotContainer().getLift().sysIdQuasistatic(Direction.
                                 * kForward)); pilot.whileTrue(ButtonType.POV_S,
                                 * Robot.getRobotContainer().getLift().sysIdQuasistatic(Direction.
                                 * kReverse));
                                 * 
                                 * pilot.whileTrue(ButtonType.POV_E,
                                 * Robot.getRobotContainer().getLift().sysIdDynamic(Direction.
                                 * kForward)); pilot.whileTrue(ButtonType.POV_W,
                                 * Robot.getRobotContainer().getLift().sysIdDynamic(Direction.
                                 * kReverse));
                                 */
                                /*
                                 * pilot.whileTrue(ButtonType.A, new WristGoToTarget(10,
                                 * HighAltitudeConstants.WRIST_DRIVE_SPEED));
                                 * 
                                 * pilot.whileTrue(ButtonType.B, new WristGoToTarget(-10,
                                 * HighAltitudeConstants.WRIST_DRIVE_SPEED));
                                 */

                                // pilot.whileTrue(ButtonType.Y, new TestDirectionPIDSwerve());
                                // pilot.whileTrue(ButtonType.LB, new TestDrivePIDFFSwerve(1));
                                // pilot.whileTrue(ButtonType.RB, new TestDrivePIDFFSwerve(-1));

                                // pilot.whileTrueCombo(new PathCancelCommand(), ButtonType.RB,
                                // ButtonType.LB);
                                // pilot.whileTrue(ButtonType.RB, new TestAlignWithPose());
                                /*
                                 * pilot.whileTrue(ButtonType.RB, new
                                 * PathplanToReefThenVisionPose(REEF_POSITION.BC, null, true, 1,
                                 * 1));
                                 */

                                // pilot.whileTrue(ButtonType.A, new ScoreCoral(REEF_HEIGHT.L3));

                                // pilot.whileTrue(ButtonType.B, new
                                // LiftWristGoToTargetHeight(REEF_HEIGHT.L3));

                                /*
                                 * pilot.whileTrue(ButtonType.POV_W, new AlignWithTargetPose(null,
                                 * null, true, HighAltitudeConstants.VISION_POSE_MAX_SPEED,
                                 * HighAltitudeConstants.VISION_POSE_MAX_TURN));
                                 * 
                                 * pilot.whileTrue(ButtonType.POV_E, new AlignWithTargetPose(null,
                                 * null, false, HighAltitudeConstants.VISION_POSE_MAX_SPEED,
                                 * HighAltitudeConstants.VISION_POSE_MAX_TURN));
                                 */
                                /*
                                 * pilot.whileTrue(ButtonType.POV_E,
                                 * Robot.getRobotContainer().getSwerveDriveTrain().
                                 * driveSysIdQuasistatic( Direction.kForward));
                                 * pilot.whileTrue(ButtonType.POV_W,
                                 * Robot.getRobotContainer().getSwerveDriveTrain().
                                 * driveSysIdQuasistatic( Direction.kReverse));
                                 * 
                                 * pilot.whileTrue(ButtonType.POV_N,
                                 * Robot.getRobotContainer().getSwerveDriveTrain().driveSysIdDynamic
                                 * (Direction. kForward)); pilot.whileTrue(ButtonType.POV_S,
                                 * Robot.getRobotContainer().getSwerveDriveTrain().driveSysIdDynamic
                                 * (Direction. kReverse));
                                 */

                                // pilot.whileTrue(ButtonType.B, new TestSwerve());
                                break;

                        case PoseTune:
                                pilot = new HighAltitudeJoystick(0, JoystickType.XBOX);
                                pilot.setAxisDeadzone(AxisType.LEFT_X, 0.1);
                                pilot.setAxisDeadzone(AxisType.LEFT_Y, 0.1);
                                pilot.setAxisDeadzone(AxisType.RIGHT_X, 0.1);

                                pilot.onTrue(ButtonType.BACK, new SetIsFieldOriented(true));
                                pilot.onTrue(ButtonType.START, new SetIsFieldOriented(false));

                                // A -> prueba de pose según modo (L1 sin backoff, LX con backoff,
                                // ALGAE = reef face)
                                pilot.onTrue(ButtonType.A,
                                                new frc.robot.commands.swerve.test.TestAlignWithPose(
                                                                frc.robot.stateMachines.TestTargets::currentPose,
                                                                HighAltitudeConstants.VISION_POSE_MAX_SPEED,
                                                                HighAltitudeConstants.VISION_POSE_MAX_TURN));

                                // B -> la misma prueba pero más suave (tune fino)
                                pilot.onTrue(ButtonType.B,
                                                new frc.robot.commands.swerve.test.TestAlignWithPose(
                                                                frc.robot.stateMachines.TestTargets::currentPose,
                                                                HighAltitudeConstants.VISION_POSE_MAX_SPEED
                                                                                * 0.6,
                                                                HighAltitudeConstants.VISION_POSE_MAX_TURN
                                                                                * 0.6));

                                // X -> ir a Coral Station (NEAR/MIDDLE/FAR por variante 0..2; lado
                                // actual L/R)
                                pilot.onTrue(ButtonType.X,
                                                new frc.robot.commands.swerve.autonomous.offSeason.DriveToCoralStation(
                                                                false,
                                                                frc.robot.stateMachines.TestTargets
                                                                                .coralStationPosFromVariant(),
                                                                Robot.isLeftMode(), // true = Left
                                                                                    // station,
                                                                                    // false = Right
                                                                HighAltitudeConstants.VISION_POSE_MAX_SPEED,
                                                                HighAltitudeConstants.VISION_POSE_MAX_TURN));

                                // Cancel combo por seguridad
                                pilot.whileTrueCombo(new PathCancelCommand(), ButtonType.LS,
                                                ButtonType.RS);
                                break;



                        default:
                                break;

                }
                switch (HighAltitudeConstants.CURRENT_COPILOT) {

                        case Pato:
                                copilot = new HighAltitudeJoystick(1, JoystickType.XBOX);

                                copilot.setAxisDeadzone(AxisType.LEFT_X, 0.1);
                                copilot.setAxisDeadzone(AxisType.LEFT_Y, 0.1);
                                copilot.setAxisDeadzone(AxisType.RIGHT_X, 0.1);


                                Supplier<GameMode> mode =
                                                () -> Robot.getRobotContainer().getGameMode();


                                // ==== Cambio de modo & Toggle Manual ====
                                copilot.onTrue(ButtonType.START, new NextModeCommand());

                                copilot.onTrue(ButtonType.BACK, new NextModeCommand());

                                copilot.onTrueCombo(new ToggleManualCommand(true), // START+BACK =
                                                                                   // //
                                                                                   // Toggle MANUAL
                                                ButtonType.START, ButtonType.BACK);
                                copilot.onTrueCombo(new ToggleManualCommand(true), ButtonType.START,
                                                ButtonType.BACK);


                                // =========================================================================
                                // MISMAS TECLAS, ACCIONES DIFERENTES SEGÚN MODO (poses Lift/Wrist)
                                // =========================================================================

                                // A: L1 Intake / Intake Rear / Algae Intake Floor / L2 (manual)
                                copilot.whileTrue(ButtonType.A, OIHelpers.onlyInMode(mode,
                                                GameMode.CORAL_L1, new SetSelectedPoseIdxCommand(
                                                                HighAltitudeConstants.PoseIdx.L1_INTAKE)));
                                copilot.whileTrue(ButtonType.A, OIHelpers.onlyInMode(mode,
                                                GameMode.CORAL_LX, new SetSelectedPoseIdxCommand(
                                                                HighAltitudeConstants.PoseIdx.INTAKE_REAR)));
                                copilot.whileTrue(ButtonType.A, OIHelpers.onlyInMode(mode,
                                                GameMode.ALGAE, new SetSelectedPoseIdxCommand(
                                                                HighAltitudeConstants.PoseIdx.ALGAE_INTAKE_FLOOR)));
                                copilot.whileTrue(ButtonType.A, OIHelpers.onlyInMode(mode,
                                                GameMode.MANUAL, new LiftWristGoToPose(
                                                                HighAltitudeConstants.PoseIdx.L2)));

                                // B: L1 Score / L4 / Processor Score / L1 Intake (manual)
                                copilot.whileTrue(ButtonType.B, OIHelpers.onlyInMode(mode,
                                                GameMode.CORAL_L1, new SetSelectedPoseIdxCommand(
                                                                HighAltitudeConstants.PoseIdx.L1_SCORE)));
                                copilot.whileTrue(ButtonType.B, OIHelpers.onlyInMode(mode,
                                                GameMode.CORAL_LX, new SetSelectedPoseIdxCommand(
                                                                HighAltitudeConstants.PoseIdx.L4)));
                                copilot.whileTrue(ButtonType.B, OIHelpers.onlyInMode(mode,
                                                GameMode.ALGAE, new SetSelectedPoseIdxCommand(
                                                                HighAltitudeConstants.PoseIdx.PROCESSOR_SCORE)));
                                copilot.whileTrue(ButtonType.B, OIHelpers.onlyInMode(mode,
                                                GameMode.MANUAL, new LiftWristGoToPose(
                                                                HighAltitudeConstants.PoseIdx.L1_INTAKE)));

                                // X: L1 Score / L2 / Algae Hold / L3 (manual)
                                copilot.whileTrue(ButtonType.X, OIHelpers.onlyInMode(mode,
                                                GameMode.CORAL_L1, new SetSelectedPoseIdxCommand(
                                                                HighAltitudeConstants.PoseIdx.L1_SCORE)));
                                copilot.whileTrue(ButtonType.X, OIHelpers.onlyInMode(mode,
                                                GameMode.CORAL_LX, new SetSelectedPoseIdxCommand(
                                                                HighAltitudeConstants.PoseIdx.L2)));
                                copilot.whileTrue(ButtonType.X, OIHelpers.onlyInMode(mode,
                                                GameMode.ALGAE, new SetSelectedPoseIdxCommand(
                                                                HighAltitudeConstants.PoseIdx.ALGAE_HOLD)));
                                copilot.whileTrue(ButtonType.X, OIHelpers.onlyInMode(mode,
                                                GameMode.MANUAL, new LiftWristGoToPose(
                                                                HighAltitudeConstants.PoseIdx.L3)));

                                // Y: L1_INTAKE / L3 / Net PrePos / L4 (manual)
                                copilot.whileTrue(ButtonType.Y, OIHelpers.onlyInMode(mode,
                                                GameMode.CORAL_L1, new SetSelectedPoseIdxCommand(
                                                                HighAltitudeConstants.PoseIdx.L1_INTAKE)));
                                copilot.whileTrue(ButtonType.Y, OIHelpers.onlyInMode(mode,
                                                GameMode.CORAL_LX, new SetSelectedPoseIdxCommand(
                                                                HighAltitudeConstants.PoseIdx.L3)));
                                copilot.whileTrue(ButtonType.Y, OIHelpers.onlyInMode(mode,
                                                GameMode.ALGAE, new SetSelectedPoseIdxCommand(
                                                                HighAltitudeConstants.PoseIdx.NET_PREPOS)));
                                copilot.whileTrue(ButtonType.Y, OIHelpers.onlyInMode(mode,
                                                GameMode.MANUAL, new LiftWristGoToPose(
                                                                HighAltitudeConstants.PoseIdx.L4)));

                                // RB/LB: pruebas de Algae Removal L3/L2 en modo ALGA (mismo botón
                                // en otros modos = no-op)
                                copilot.whileTrue(ButtonType.RB, OIHelpers.onlyInMode(mode,
                                                GameMode.ALGAE, new LiftWristGoToPose(
                                                                HighAltitudeConstants.PoseIdx.ALGAE_REMOVE_L3)));


                                copilot.whileTrue(ButtonType.LB, OIHelpers.onlyInMode(mode,
                                                GameMode.ALGAE, new LiftWristGoToPose(
                                                                HighAltitudeConstants.PoseIdx.ALGAE_REMOVE_L2)));
                                copilot.whileTrue(ButtonType.LB,
                                                OIHelpers.onlyInMode(mode, GameMode.CORAL_LX,
                                                                new LiftWristGoToSelectedPose()));
                                copilot.whileTrue(ButtonType.LB,
                                                OIHelpers.onlyInMode(mode, GameMode.CORAL_L1,
                                                                new LiftWristGoToSelectedPose()));

                                copilot.onTrue(ButtonType.LB, OIHelpers.onlyInMode(mode,
                                                GameMode.CORAL_L1, new IntakeAlgaeAuto()));


                                // POV E/W: en ALGA, probar NET pre/score rápidamente
                                copilot.whileTrue(ButtonType.POV_E, OIHelpers.onlyInMode(mode,
                                                GameMode.ALGAE, new LiftWristGoToPose(
                                                                HighAltitudeConstants.PoseIdx.NET_PREPOS)));
                                copilot.whileTrue(ButtonType.POV_W, OIHelpers.onlyInMode(mode,
                                                GameMode.ALGAE, new LiftWristGoToPose(
                                                                HighAltitudeConstants.PoseIdx.NET_SCORE)));

                                // Triggers en ALGA: intake/eject (si ya los tenías así)
                                copilot.whileTrue(ButtonType.LT, new ScoreGamePiece(
                                                HighAltitudeConstants.GRIPPER_IN_SPEED));

                                copilot.onTrue(ButtonType.RT, OIHelpers.onlyInMode(mode,
                                                GameMode.ALGAE, new IntakeAlgaeAuto()));

                                copilot.onTrue(ButtonType.RS,
                                                new LiftWristGoToPose(PoseIdx.INTAKE_REAR));
                                // Utilidades
                                copilot.onTrue(ButtonType.LS, new ResetLiftEncoders());
                                break;


                        case PoseTune:
                                copilot = new HighAltitudeJoystick(1, JoystickType.XBOX);

                                // Cambiar GameMode (ciclo): CORAL_L1 -> CORAL_LX -> ALGAE ->
                                // MANUAL...
                                copilot.onTrue(ButtonType.START,
                                                new frc.robot.stateMachines.NextModeCommand());

                                // Toggle manual rápido, si lo quieres
                                copilot.onTrue(ButtonType.BACK,
                                                new frc.robot.stateMachines.ToggleManualCommand());

                                // Lado L/R (afecta branch en L1/LX y CoralStation)
                                copilot.whileTrue(ButtonType.POV_W, new SetLeftMode(true));
                                copilot.whileTrue(ButtonType.POV_E, new SetLeftMode(false));

                                // Variante:
                                // - L1/LX: 0..3 => (A/B),(C/D),(E/F),(G/H) según lado
                                // - ALGAE: 0..5 => BC,BR,FR,FC,FL,BL
                                copilot.onTrue(ButtonType.A,
                                                new edu.wpi.first.wpilibj2.command.InstantCommand(
                                                                () -> frc.robot.stateMachines.TestTargets
                                                                                .setCaralho_var(0)));
                                copilot.onTrue(ButtonType.X,
                                                new edu.wpi.first.wpilibj2.command.InstantCommand(
                                                                () -> frc.robot.stateMachines.TestTargets
                                                                                .setCaralho_var(1)));
                                copilot.onTrue(ButtonType.Y,
                                                new edu.wpi.first.wpilibj2.command.InstantCommand(
                                                                () -> frc.robot.stateMachines.TestTargets
                                                                                .setCaralho_var(2)));
                                copilot.onTrue(ButtonType.B,
                                                new edu.wpi.first.wpilibj2.command.InstantCommand(
                                                                () -> frc.robot.stateMachines.TestTargets
                                                                                .setCaralho_var(3)));
                                // extras para ALGAE (caras faltantes):
                                copilot.onTrue(ButtonType.LB,
                                                new edu.wpi.first.wpilibj2.command.InstantCommand(
                                                                () -> frc.robot.stateMachines.TestTargets
                                                                                .setCaralho_var(4))); // FL
                                copilot.onTrue(ButtonType.RB,
                                                new edu.wpi.first.wpilibj2.command.InstantCommand(
                                                                () -> frc.robot.stateMachines.TestTargets
                                                                                .setCaralho_var(5))); // BL
                                break;


                        case ItaiAndGomezButChambingButCompetionButIsLeonButIsREEFSCAPE:

                                copilot = new HighAltitudeJoystick(1, JoystickType.XBOX);

                                copilot.onTrue(ButtonType.BACK, new SetCoralMode(false)); // Algae
                                                                                          // Mode
                                copilot.onTrue(ButtonType.START, new SetCoralMode(true)); // Coral
                                                                                          // Mode

                                copilot.whileTrue(ButtonType.LB, new ScoreGamePiece(
                                                HighAltitudeConstants.GRIPPER_IN_SPEED)); // Score
                                                                                          // Game
                                                                                          // Piece
                                copilot.whileTrue(ButtonType.RB, new IntakeAlgae()); // Intake Algae
                                                                                     // / Reverse
                                                                                     // Coral

                                copilot.whileTrue(ButtonType.POV_N, new LiftUp());
                                copilot.whileTrue(ButtonType.POV_S, new LiftDown());

                                copilot.whileTrue(ButtonType.POV_E, new WristGoToTarget(0, 0.1));
                                copilot.whileTrue(ButtonType.POV_W, new WristGoToTarget(45, 0.1));

                                break;

                        /*
                         * case LiftWristTest:
                         * 
                         * copilot = new HighAltitudeJoystick(1, JoystickType.XBOX);
                         * 
                         * copilot.setAxisDeadzone(AxisType.LEFT_X, 0.1);
                         * copilot.setAxisDeadzone(AxisType.LEFT_Y, 0.1);
                         * copilot.setAxisDeadzone(AxisType.RIGHT_X, 0.1);
                         * 
                         * // Supplier del modo actual (State) Supplier<GameMode> mode = () ->
                         * Robot.getRobotContainer().getGameMode();
                         * 
                         * // ==== Cambio de modo & Toggle Manual ====
                         * copilot.onTrue(ButtonType.START, new NextModeCommand()); // Ciclar //
                         * modo → copilot.onTrue(ButtonType.BACK, new NextModeCommand()); // …en
                         * ambos // botones // para test copilot.onTrueCombo(new
                         * ToggleManualCommand(true), // START+BACK = // Toggle MANUAL
                         * ButtonType.START, ButtonType.BACK);
                         * 
                         * //
                         * =========================================================================
                         * // MISMA TECLA, ACCIONES DIFERENTES SEGÚN MODO (guards con // onlyInMode)
                         * //
                         * =========================================================================
                         * // A: L1 Intake / Intake Rear / Algae Intake Floor / L2 (manual)
                         * copilot.whileTrue(ButtonType.A, onlyInMode(mode, GameMode.CORAL_L1, new
                         * LiftWristGoToPose(PoseIdx.L1_INTAKE))); copilot.whileTrue(ButtonType.A,
                         * onlyInMode(mode, GameMode.CORAL_LX, new
                         * LiftWristGoToPose(PoseIdx.INTAKE_REAR))); copilot.whileTrue(ButtonType.A,
                         * onlyInMode(mode, GameMode.ALGAE, new
                         * LiftWristGoToPose(PoseIdx.ALGAE_INTAKE_FLOOR)));
                         * copilot.whileTrue(ButtonType.A, onlyInMode(mode, GameMode.MANUAL, new
                         * LiftWristGoToPose(PoseIdx.L2)));
                         * 
                         * // B: L1 Score / L4 / Processor Score / L1 Intake (manual)
                         * copilot.whileTrue(ButtonType.B, onlyInMode(mode, GameMode.CORAL_L1, new
                         * LiftWristGoToPose(PoseIdx.L1_SCORE))); copilot.whileTrue(ButtonType.B,
                         * onlyInMode(mode, GameMode.CORAL_LX, new LiftWristGoToPose(PoseIdx.L4)));
                         * copilot.whileTrue(ButtonType.B, onlyInMode(mode, GameMode.ALGAE, new
                         * LiftWristGoToPose(PoseIdx.PROCESSOR_SCORE)));
                         * copilot.whileTrue(ButtonType.B, onlyInMode(mode, GameMode.MANUAL, new
                         * LiftWristGoToPose(PoseIdx.L1_INTAKE)));
                         * 
                         * // X: L2 (test desde Coral L1) / L2 / Algae Hold / L3 (manual)
                         * copilot.whileTrue(ButtonType.X, onlyInMode(mode, GameMode.CORAL_L1, new
                         * LiftWristGoToPose(PoseIdx.L2))); copilot.whileTrue(ButtonType.X,
                         * onlyInMode(mode, GameMode.CORAL_LX, new LiftWristGoToPose(PoseIdx.L2)));
                         * copilot.whileTrue(ButtonType.X, onlyInMode(mode, GameMode.ALGAE, new
                         * LiftWristGoToPose(PoseIdx.ALGAE_HOLD))); copilot.whileTrue(ButtonType.X,
                         * onlyInMode(mode, GameMode.MANUAL, new LiftWristGoToPose(PoseIdx.L3)));
                         * 
                         * // Y: L3 (test desde Coral L1) / L3 / Net PrePos / L4 (manual)
                         * copilot.whileTrue(ButtonType.Y, onlyInMode(mode, GameMode.CORAL_L1, new
                         * LiftWristGoToPose(PoseIdx.L3))); copilot.whileTrue(ButtonType.Y,
                         * onlyInMode(mode, GameMode.CORAL_LX, new LiftWristGoToPose(PoseIdx.L3)));
                         * copilot.whileTrue(ButtonType.Y, onlyInMode(mode, GameMode.ALGAE, new
                         * LiftWristGoToPose(PoseIdx.NET_PREPOS))); copilot.whileTrue(ButtonType.Y,
                         * onlyInMode(mode, GameMode.MANUAL, new LiftWristGoToPose(PoseIdx.L4)));
                         * 
                         * // RB/LB: pruebas de Algae Removal L3/L2 en modo ALGA (mismo botón // en
                         * otros // modos no hace nada) copilot.whileTrue(ButtonType.RB,
                         * onlyInMode(mode, GameMode.ALGAE, new
                         * LiftWristGoToPose(PoseIdx.ALGAE_REMOVE_L3)));
                         * copilot.whileTrue(ButtonType.LB, onlyInMode(mode, GameMode.ALGAE, new
                         * LiftWristGoToPose(PoseIdx.ALGAE_REMOVE_L2)));
                         * 
                         * // POV: en ALGA probar NET pre/score rápidamente
                         * copilot.whileTrue(ButtonType.POV_E, onlyInMode(mode, GameMode.ALGAE, new
                         * LiftWristGoToPose(PoseIdx.NET_PREPOS)));
                         * copilot.whileTrue(ButtonType.POV_W, onlyInMode(mode, GameMode.ALGAE, new
                         * LiftWristGoToPose(PoseIdx.NET_SCORE)));
                         * 
                         * // Triggers: probar IntakeUntilCurrentAlgae solo en modo ALGA
                         * copilot.whileTrue(ButtonType.LT, onlyInMode(mode, GameMode.ALGAE, new
                         * ScoreGamePiece( HighAltitudeConstants.GRIPPER_IN_SPEED)));
                         * 
                         * copilot.onTrue(ButtonType.RT, onlyInMode(mode, GameMode.ALGAE, new
                         * IntakeAlgaeAuto())); // (opcional) RT para subir a HOLD inmediatamente
                         * break;
                         */


                        default:

                                break;
                }
        }

        public static OI getInstance() {
                if (instance == null) {
                        instance = new OI();
                }
                return instance;
        }

        public double getDefaultSwerveDriveSpeed() {

                switch (HighAltitudeConstants.CURRENT_PILOT) {

                        case DefaultUser:
                                return -pilot.getAxis(AxisType.LEFT_Y);

                        case Joakin:
                                return -pilot.getAxis(AxisType.LEFT_Y);

                        default:
                                return -pilot.getAxis(AxisType.LEFT_Y);

                }
        }

        public double getDefaultSwerveDriveStrafe() {

                switch (HighAltitudeConstants.CURRENT_PILOT) {

                        case DefaultUser:
                                return -pilot.getAxis(AxisType.LEFT_X);

                        case Joakin:
                                return -pilot.getAxis(AxisType.LEFT_X);

                        default:
                                return -pilot.getAxis(AxisType.LEFT_X);
                }
        }

        public double getDefaultSwerveDriveTurn() {

                switch (HighAltitudeConstants.CURRENT_PILOT) {

                        case DefaultUser:
                                return -pilot.getAxis(AxisType.RIGHT_X);

                        case Joakin:
                                return -pilot.getAxis(AxisType.RIGHT_X);

                        default:
                                return -pilot.getAxis(AxisType.RIGHT_X);
                }
        }

        public HighAltitudeJoystick getPilot() {
                switch (HighAltitudeConstants.CURRENT_PILOT) {

                        case DefaultUser:
                                return pilot;

                        case Joakin:
                                return pilot;

                        default:
                                return pilot;
                }
        }

        public HighAltitudeJoystick getCopilot() {
                switch (HighAltitudeConstants.CURRENT_COPILOT) {

                        case DefaultUser:
                                return copilot;

                        case Joakin:
                                return copilot;

                        default:
                                return copilot;
                }
        }

}
