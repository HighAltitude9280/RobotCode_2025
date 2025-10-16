// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.resources.math;
// src/main/java/frc/robot/util/PoseUtil.java

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;

/**
 * Utilidades para desplazar una Pose2d en su marco local.
 * - forward > 0 avanza en la dirección del heading; forward < 0 es backoff.
 * - left > 0 desplaza hacia la izquierda del heading; left < 0 a la derecha.
 */
public final class PoseUtil {
  private PoseUtil() {
  }

  /**
   * Offset en el marco local: +forward(m), +left(m). Mantiene la misma rotación.
   */
  public static Pose2d offsetLocal(Pose2d base, double forwardMeters, double leftMeters) {
    double th = base.getRotation().getRadians();
    double dx = Math.cos(th) * forwardMeters - Math.sin(th) * leftMeters;
    double dy = Math.sin(th) * forwardMeters + Math.cos(th) * leftMeters;
    return new Pose2d(base.getX() + dx, base.getY() + dy, base.getRotation());
  }

  /** Atajo: avanzar dist metros en la dirección del heading. */
  public static Pose2d forward(Pose2d base, double meters) {
    return offsetLocal(base, +meters, 0.0);
  }

  /**
   * Atajo: retroceder dist metros (backoff) en la dirección contraria al heading.
   */
  public static Pose2d backoff(Pose2d base, double meters) {
    return offsetLocal(base, -meters, 0.0);
  }

  /** Backoff en pulgadas (positivo) = retroceder esa distancia. */
  public static Pose2d backoffInches(Pose2d base, double inches) {
    return backoff(base, Units.inchesToMeters(inches));
  }
}
