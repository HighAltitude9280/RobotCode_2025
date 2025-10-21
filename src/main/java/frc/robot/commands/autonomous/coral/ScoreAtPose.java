// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.coral;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.HighAltitudeConstants;
// Importa la clase interna PoseIdx
import frc.robot.HighAltitudeConstants.PoseIdx;
import frc.robot.commands.extensor.gripper.manual.ScoreGamePiece;
import frc.robot.commands.extensor.lift.control.LiftDefaultCommand;
import frc.robot.commands.extensor.lift.control.LiftGoToTarget;
import frc.robot.commands.extensor.wrist.control.WristGoToTarget;

/**
 * Comando secuencial para moverse a una pose de anotación (definida por PoseIdx), soltar la pieza,
 * y regresar a una pose segura (L1_SCORE). * Este comando usa un 'switch' para manejar secuencias
 * especiales, como la de L4 que requiere un offset en el wrist.
 */
public class ScoreAtPose extends SequentialCommandGroup {

        /**
         * Crea una nueva secuencia de anotación.
         *
         * @param scorePoseIdx El índice de la pose de anotación (ej. PoseIdx.L2, PoseIdx.L4,
         *        PoseIdx.NET_SCORE).
         */
        public ScoreAtPose(int scorePoseIdx) {

                // --- 1. Definir Objetivos (Targets) ---

                // Objetivos de "Arriba" (Anotación)
                double liftTargetUp = HighAltitudeConstants.liftForPose(scorePoseIdx);
                double wristTargetUp = HighAltitudeConstants.wristForPose(scorePoseIdx);

                // Objetivos de "Abajo" (Regreso/Seguro)
                // Usamos L1_SCORE como la posición de regreso, similar a REEF_HEIGHT.BOTTOM
                int poseDownIdx = PoseIdx.L1_SCORE;
                double liftTargetDown = HighAltitudeConstants.liftForPose(poseDownIdx);
                double wristTargetDown = HighAltitudeConstants.wristForPose(poseDownIdx);

                // --- 2. Construir Secuencia de Comandos (con "casos") ---

                // Usamos un switch para manejar casos especiales de anotación
                switch (scorePoseIdx) {
                        case PoseIdx.L4:
                                // CASO ESPECIAL: L4 (TOP)
                                // Primero sube a L3, luego a L4, y al final aplica offset de wrist

                                // --- Obtenemos las alturas que necesitamos ---
                                // 'liftTargetUp' ya tiene la altura de L4 (viene de la línea 19)
                                double liftTargetL3 = HighAltitudeConstants.liftForPose(PoseIdx.L3);


                                addCommands(
                                                // 1. Ir a altura L3
                                                new LiftGoToTarget(
                                                                HighAltitudeConstants.LIFT_MAX_POWER,
                                                                liftTargetL3, // <-- CORREGIDO
                                                                HighAltitudeConstants.LIFT_ARRIVE_OFFSET),

                                                // 2. Ir de L3 a L4
                                                new LiftGoToTarget(
                                                                HighAltitudeConstants.LIFT_MAX_POWER,
                                                                liftTargetUp, // <-- 'liftTargetUp'
                                                                              // es la altura de L4
                                                                HighAltitudeConstants.LIFT_ARRIVE_OFFSET),

                                                // 3. Mantener altura (L4), mover wrist a (target -
                                                // offset)
                                                new ParallelRaceGroup(new LiftDefaultCommand(
                                                                HighAltitudeConstants.LIFT_MAX_POWER,
                                                                HighAltitudeConstants.LIFT_ARRIVE_OFFSET),
                                                                new WristGoToTarget(
                                                                                wristTargetUp - HighAltitudeConstants.WRIST_L4_OFFSET, // El
                                                                                                                                       // offset
                                                                                HighAltitudeConstants.WRIST_DRIVE_SPEED)),

                                                // 4. Mantener altura (L4), anotar (expulsar pieza)
                                                new ParallelRaceGroup(new ScoreGamePiece(
                                                                HighAltitudeConstants.GRIPPER_IN_SPEED)
                                                                                .withTimeout(0.5),
                                                                new LiftDefaultCommand(
                                                                                HighAltitudeConstants.LIFT_MAX_POWER,
                                                                                HighAltitudeConstants.LIFT_ARRIVE_OFFSET)),

                                                // 5. Mantener altura (L4), mover wrist a posición
                                                // "abajo"
                                                new ParallelRaceGroup(new LiftDefaultCommand(
                                                                HighAltitudeConstants.LIFT_MAX_POWER,
                                                                HighAltitudeConstants.LIFT_ARRIVE_OFFSET),
                                                                new WristGoToTarget(wristTargetDown,
                                                                                HighAltitudeConstants.WRIST_DRIVE_SPEED)),

                                                // 6. Bajar lift a posición "abajo"
                                                new LiftGoToTarget(
                                                                HighAltitudeConstants.LIFT_MAX_POWER_GOING_DOWN,
                                                                liftTargetDown,
                                                                HighAltitudeConstants.LIFT_ARRIVE_OFFSET));
                                break;

                        // CASO DEFAULT: Para todas las demás posiciones de anotación
                        // (Coral L1, L2, L3 y Alga Processor, Net)

                        case PoseIdx.L1_SCORE:
                        case PoseIdx.L2:
                                addCommands(
                                                // 1. Ir a altura de anotación
                                                new LiftGoToTarget(
                                                                HighAltitudeConstants.LIFT_MAX_POWER,
                                                                liftTargetUp,
                                                                HighAltitudeConstants.LIFT_ARRIVE_OFFSET),

                                                // 2. Mantener altura, mover wrist a ángulo de
                                                // anotación
                                                new ParallelRaceGroup(new LiftDefaultCommand(
                                                                HighAltitudeConstants.LIFT_MAX_POWER,
                                                                HighAltitudeConstants.LIFT_ARRIVE_OFFSET),
                                                                new WristGoToTarget(wristTargetUp, // <--
                                                                                                   // Sin
                                                                                                   // offset
                                                                                HighAltitudeConstants.WRIST_DRIVE_SPEED)),

                                                // 3. Mantener altura, anotar (expulsar pieza)
                                                new ParallelRaceGroup(new ScoreGamePiece(
                                                                HighAltitudeConstants.GRIPPER_IN_SPEED)
                                                                                .withTimeout(0.5),
                                                                new LiftDefaultCommand(
                                                                                HighAltitudeConstants.LIFT_MAX_POWER,
                                                                                HighAltitudeConstants.LIFT_ARRIVE_OFFSET)),

                                                // 4. Mantener altura, mover wrist a posición
                                                // "abajo"
                                                new ParallelRaceGroup(new LiftDefaultCommand(
                                                                HighAltitudeConstants.LIFT_MAX_POWER,
                                                                HighAltitudeConstants.LIFT_ARRIVE_OFFSET),
                                                                new WristGoToTarget(wristTargetDown,
                                                                                HighAltitudeConstants.WRIST_DRIVE_SPEED)),

                                                // 5. Bajar lift a posición "abajo"
                                                new LiftGoToTarget(
                                                                HighAltitudeConstants.LIFT_MAX_POWER_GOING_DOWN,
                                                                liftTargetDown,
                                                                HighAltitudeConstants.LIFT_ARRIVE_OFFSET));
                                break;
                        case PoseIdx.L3:
                                addCommands(
                                                // 1. Ir a altura de anotación
                                                new LiftGoToTarget(
                                                                HighAltitudeConstants.LIFT_MAX_POWER,
                                                                liftTargetUp,
                                                                HighAltitudeConstants.LIFT_ARRIVE_OFFSET),

                                                // 2. Mantener altura, mover wrist a ángulo de
                                                // anotación
                                                new ParallelRaceGroup(new LiftDefaultCommand(
                                                                HighAltitudeConstants.LIFT_MAX_POWER,
                                                                HighAltitudeConstants.LIFT_ARRIVE_OFFSET),
                                                                new WristGoToTarget(wristTargetUp, // <--
                                                                                                   // Sin
                                                                                                   // offset
                                                                                HighAltitudeConstants.WRIST_DRIVE_SPEED)),

                                                // 3. Mantener altura, anotar (expulsar pieza)
                                                new ParallelRaceGroup(new ScoreGamePiece(
                                                                HighAltitudeConstants.GRIPPER_IN_SPEED)
                                                                                .withTimeout(0.5),
                                                                new LiftDefaultCommand(
                                                                                HighAltitudeConstants.LIFT_MAX_POWER,
                                                                                HighAltitudeConstants.LIFT_ARRIVE_OFFSET)),

                                                // 4. Mantener altura, mover wrist a posición
                                                // "abajo"
                                                new ParallelRaceGroup(new LiftDefaultCommand(
                                                                HighAltitudeConstants.LIFT_MAX_POWER,
                                                                HighAltitudeConstants.LIFT_ARRIVE_OFFSET),
                                                                new WristGoToTarget(wristTargetDown,
                                                                                HighAltitudeConstants.WRIST_DRIVE_SPEED)),

                                                // 5. Bajar lift a posición "abajo"
                                                new LiftGoToTarget(
                                                                HighAltitudeConstants.LIFT_MAX_POWER_GOING_DOWN,
                                                                liftTargetDown,
                                                                HighAltitudeConstants.LIFT_ARRIVE_OFFSET));
                                break;
                        case PoseIdx.PROCESSOR_SCORE: // <-- Funciona para Alga
                                addCommands(
                                                // 1. Ir a altura de anotación
                                                new LiftGoToTarget(
                                                                HighAltitudeConstants.LIFT_MAX_POWER,
                                                                liftTargetUp,
                                                                HighAltitudeConstants.LIFT_ARRIVE_OFFSET),

                                                // 2. Mantener altura, mover wrist a ángulo de
                                                // anotación
                                                new ParallelRaceGroup(new LiftDefaultCommand(
                                                                HighAltitudeConstants.LIFT_MAX_POWER,
                                                                HighAltitudeConstants.LIFT_ARRIVE_OFFSET),
                                                                new WristGoToTarget(wristTargetUp, // <--
                                                                                                   // Sin
                                                                                                   // offset
                                                                                HighAltitudeConstants.WRIST_DRIVE_SPEED)),

                                                // 3. Mantener altura, anotar (expulsar pieza)
                                                new ParallelRaceGroup(new ScoreGamePiece(
                                                                HighAltitudeConstants.GRIPPER_IN_SPEED)
                                                                                .withTimeout(0.5),
                                                                new LiftDefaultCommand(
                                                                                HighAltitudeConstants.LIFT_MAX_POWER,
                                                                                HighAltitudeConstants.LIFT_ARRIVE_OFFSET)),

                                                // 4. Mantener altura, mover wrist a posición
                                                // "abajo"
                                                new ParallelRaceGroup(new LiftDefaultCommand(
                                                                HighAltitudeConstants.LIFT_MAX_POWER,
                                                                HighAltitudeConstants.LIFT_ARRIVE_OFFSET),
                                                                new WristGoToTarget(wristTargetDown,
                                                                                HighAltitudeConstants.WRIST_DRIVE_SPEED)),

                                                // 5. Bajar lift a posición "abajo"
                                                new LiftGoToTarget(
                                                                HighAltitudeConstants.LIFT_MAX_POWER_GOING_DOWN,
                                                                liftTargetDown,
                                                                HighAltitudeConstants.LIFT_ARRIVE_OFFSET));
                                break;
                        case PoseIdx.NET_SCORE: // <-- Funciona para Alga
                                addCommands(
                                                // 1. Ir a altura de anotación
                                                new LiftGoToTarget(
                                                                HighAltitudeConstants.LIFT_MAX_POWER,
                                                                liftTargetUp,
                                                                HighAltitudeConstants.LIFT_ARRIVE_OFFSET),

                                                // 2. Mantener altura, mover wrist a ángulo de
                                                // anotación
                                                new ParallelRaceGroup(new LiftDefaultCommand(
                                                                HighAltitudeConstants.LIFT_MAX_POWER,
                                                                HighAltitudeConstants.LIFT_ARRIVE_OFFSET),
                                                                new WristGoToTarget(wristTargetUp, // <--
                                                                                                   // Sin
                                                                                                   // offset
                                                                                HighAltitudeConstants.WRIST_DRIVE_SPEED)),

                                                // 3. Mantener altura, anotar (expulsar pieza)
                                                new ParallelRaceGroup(new ScoreGamePiece(
                                                                HighAltitudeConstants.GRIPPER_IN_SPEED)
                                                                                .withTimeout(0.5),
                                                                new LiftDefaultCommand(
                                                                                HighAltitudeConstants.LIFT_MAX_POWER,
                                                                                HighAltitudeConstants.LIFT_ARRIVE_OFFSET)),

                                                // 4. Mantener altura, mover wrist a posición
                                                // "abajo"
                                                new ParallelRaceGroup(new LiftDefaultCommand(
                                                                HighAltitudeConstants.LIFT_MAX_POWER,
                                                                HighAltitudeConstants.LIFT_ARRIVE_OFFSET),
                                                                new WristGoToTarget(wristTargetDown,
                                                                                HighAltitudeConstants.WRIST_DRIVE_SPEED)),

                                                // 5. Bajar lift a posición "abajo"
                                                new LiftGoToTarget(
                                                                HighAltitudeConstants.LIFT_MAX_POWER_GOING_DOWN,
                                                                liftTargetDown,
                                                                HighAltitudeConstants.LIFT_ARRIVE_OFFSET));
                                break;
                        case PoseIdx.NET_PREPOS:
                                addCommands(
                                                // 1. Ir a altura de anotación
                                                new LiftGoToTarget(
                                                                HighAltitudeConstants.LIFT_MAX_POWER,
                                                                liftTargetUp,
                                                                HighAltitudeConstants.LIFT_ARRIVE_OFFSET),

                                                // 2. Mantener altura, mover wrist a ángulo de
                                                // anotación
                                                new ParallelRaceGroup(new LiftDefaultCommand(
                                                                HighAltitudeConstants.LIFT_MAX_POWER,
                                                                HighAltitudeConstants.LIFT_ARRIVE_OFFSET),
                                                                new WristGoToTarget(wristTargetUp, // <--
                                                                                                   // Sin
                                                                                                   // offset
                                                                                HighAltitudeConstants.WRIST_DRIVE_SPEED)),

                                                // 3. Mantener altura
                                                new ParallelRaceGroup(new WaitCommand(0.5),
                                                                new LiftDefaultCommand(
                                                                                HighAltitudeConstants.LIFT_MAX_POWER,
                                                                                HighAltitudeConstants.LIFT_ARRIVE_OFFSET)));
                                break;
                        // ... puedes añadir otros casos estándar aquí ...
                        default:
                                // Secuencia estándar sin offset
                                addCommands(
                                                // 1. Ir a altura de anotación
                                                new LiftGoToTarget(
                                                                HighAltitudeConstants.LIFT_MAX_POWER,
                                                                liftTargetUp,
                                                                HighAltitudeConstants.LIFT_ARRIVE_OFFSET),

                                                // 2. Mantener altura, mover wrist a ángulo de
                                                // anotación
                                                new ParallelRaceGroup(new LiftDefaultCommand(
                                                                HighAltitudeConstants.LIFT_MAX_POWER,
                                                                HighAltitudeConstants.LIFT_ARRIVE_OFFSET),
                                                                new WristGoToTarget(wristTargetUp, // <--
                                                                                                   // Sin
                                                                                                   // offset
                                                                                HighAltitudeConstants.WRIST_DRIVE_SPEED)),

                                                // 3. Mantener altura, anotar (expulsar pieza)
                                                new ParallelRaceGroup(new ScoreGamePiece(
                                                                HighAltitudeConstants.GRIPPER_IN_SPEED)
                                                                                .withTimeout(0.5),
                                                                new LiftDefaultCommand(
                                                                                HighAltitudeConstants.LIFT_MAX_POWER,
                                                                                HighAltitudeConstants.LIFT_ARRIVE_OFFSET)),

                                                // 4. Mantener altura, mover wrist a posición
                                                // "abajo"
                                                new ParallelRaceGroup(new LiftDefaultCommand(
                                                                HighAltitudeConstants.LIFT_MAX_POWER,
                                                                HighAltitudeConstants.LIFT_ARRIVE_OFFSET),
                                                                new WristGoToTarget(wristTargetDown,
                                                                                HighAltitudeConstants.WRIST_DRIVE_SPEED)),

                                                // 5. Bajar lift a posición "abajo"
                                                new LiftGoToTarget(
                                                                HighAltitudeConstants.LIFT_MAX_POWER_GOING_DOWN,
                                                                liftTargetDown,
                                                                HighAltitudeConstants.LIFT_ARRIVE_OFFSET));
                                break;

                }
        }
}
