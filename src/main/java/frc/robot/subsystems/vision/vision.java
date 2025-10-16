// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HighAltitudeConstants;

public class Vision extends SubsystemBase {

  // ==== TUNABLES PARA EL MODO RELIABLE DE ALINEACIÓN ====
  private static final double ALIGN_TTL_SEC = 0.35; // cuánto “aguanta” el último target fresco
  private static final double EPS = 1e-9; // para comparaciones

  ArrayList<PhotonCamera> cams;
  ArrayList<PhotonPoseEstimator> poseEstimators;
  ArrayList<List<PhotonPipelineResult>> results;

  // Cache del mejor target por cámara (para alineación robusta)
  private PhotonTrackedTarget[] lastGoodTarget; // por cámara
  private double[] lastGoodTs; // timestamp (FPGATime) del target cacheado
  private int lastChosenCam = -1; // debug: última cámara elegida para alinear

  /** Creates a new vision. */
  public Vision() {
    cams = new ArrayList<>();
    poseEstimators = new ArrayList<>();
    results = new ArrayList<>();

    AprilTagFieldLayout fieldLayout;
    // Si usas un layout custom, descomenta el try/catch y comenta la siguiente línea.
    fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // Construcción alineada por índice
    for (int i = 0; i < HighAltitudeConstants.CAMERA_NAMES.length; i++) {
      var name = HighAltitudeConstants.CAMERA_NAMES[i];
      var robotToCamera = HighAltitudeConstants.CAMERA_POSITIONS[i];

      var cam = new PhotonCamera(name);
      cams.add(cam);

      var est = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          robotToCamera);

      // Fallback a single-tag cuando no haya multi-tag
      est.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

      poseEstimators.add(est);
    }

    // Inicializa cache para alineación
    lastGoodTarget = new PhotonTrackedTarget[cams.size()];
    lastGoodTs = new double[cams.size()];
    for (int i = 0; i < cams.size(); i++) {
      lastGoodTarget[i] = null;
      lastGoodTs[i] = -1.0;
    }
  }

  // ========= MÉTODOS EXISTENTES (no removidos) =========

  public List<EstimatedRobotPose> getEstimatedRobotPoses() {
    var poses = new ArrayList<EstimatedRobotPose>();
    int n = Math.min(cams.size(), Math.min(results.size(), poseEstimators.size()));

    for (int i = 0; i < n; i++) {
      var resList = results.get(i);
      if (resList == null || resList.isEmpty())
        continue;

      var estimator = poseEstimators.get(i); // efectivamente final

      Optional<EstimatedRobotPose> pose = resList.stream().filter(r -> r.hasTargets()).filter(r -> {
        var t = r.getBestTarget();
        double dist = t.bestCameraToTarget.getTranslation().getNorm();
        double amb = t.poseAmbiguity;
        return dist <= HighAltitudeConstants.VISION_POSE_ESTIMATOR_MAX_DISTANCE
            && amb <= HighAltitudeConstants.VISION_POSE_ESTIMATOR_MAX_AMBIGUITY;
      }).findFirst() // o .min(Comparator.comparingDouble(r -> r.getBestTarget().poseAmbiguity))
          .flatMap(estimator::update);

      pose.ifPresent(poses::add);
    }
    return poses;
  }

  public boolean alignmentCamHasTargets() {
    if (results.isEmpty())
      return false;
    for (int i : HighAltitudeConstants.ALIGNMENT_CAMERAS) {
      if (i < 0 || i >= results.size())
        continue;
      if (!results.get(i).isEmpty() && results.get(i).get(results.get(i).size() - 1).hasTargets())
        return true;
    }
    return false;
  }

  public boolean targetsVisible() {
    if (results.isEmpty())
      return false;
    for (var camResults : results) {
      if (!camResults.isEmpty() && camResults.get(camResults.size() - 1).hasTargets())
        return true;
    }
    return false;
  }

  public int getTargetID() {
    if (results.isEmpty())
      return -1;
    else {
      for (int i : HighAltitudeConstants.ALIGNMENT_CAMERAS) {
        if (i < 0 || i >= results.size())
          continue;
        if (!results.get(i).isEmpty()) {
          var target = results.get(i).get(results.get(i).size() - 1).getBestTarget();
          if (target == null)
            continue;
          else
            return target.getFiducialId();
        }
      }
    }
    return -1;
  }

  @Override
  public void periodic() {
    results.clear();
    for (var cam : cams) {
      results.add(cam.getAllUnreadResults());
    }

    // === Actualiza cache reliable del mejor target por cámara ===
    final double now = Timer.getFPGATimestamp();
    for (int i = 0; i < results.size(); i++) {
      var resList = results.get(i);
      if (resList == null || resList.isEmpty())
        continue;

      var r = resList.get(resList.size() - 1); // frame más reciente
      if (!r.hasTargets())
        continue;

      PhotonTrackedTarget best = null;
      double bestAmb = Double.POSITIVE_INFINITY;
      double bestYawA = Double.POSITIVE_INFINITY;
      double bestDist = Double.POSITIVE_INFINITY;

      for (var t : r.getTargets()) {
        double dist = t.bestCameraToTarget.getTranslation().getNorm();
        double amb = t.poseAmbiguity;

        boolean ambOk =
            (amb < 0) || (amb <= HighAltitudeConstants.VISION_POSE_ESTIMATOR_MAX_AMBIGUITY);
        boolean distOk = dist <= HighAltitudeConstants.VISION_POSE_ESTIMATOR_MAX_DISTANCE;
        if (!(ambOk && distOk))
          continue;

        double yawAbs = Math.abs(t.getYaw());
        double ambScore = (amb < 0) ? 0.0 : amb;

        if (ambScore + EPS < bestAmb
            || (Math.abs(ambScore - bestAmb) < EPS && (yawAbs + EPS < bestYawA
                || (Math.abs(yawAbs - bestYawA) < EPS && dist + EPS < bestDist)))) {
          best = t;
          bestAmb = ambScore;
          bestYawA = yawAbs;
          bestDist = dist;
        }
      }

      if (best != null) {
        lastGoodTarget[i] = best;
        lastGoodTs[i] = now;
      }
    }

    /*
     * SmartDashboard.putBoolean("PV/cam0/connected", cams.size() > 0 && cams.get(0).isConnected());
     * SmartDashboard.putBoolean("PV/cam1/connected", cams.size() > 1 && cams.get(1).isConnected());
     * SmartDashboard.putNumber("PV/cam0/unread", results.size() > 0 && results.get(0) != null ?
     * results.get(0).size() : -1); SmartDashboard.putNumber("PV/cam1/unread", results.size() > 1 &&
     * results.get(1) != null ? results.get(1).size() : -1);
     * SmartDashboard.putNumber("Vision/align_lastCam", lastChosenCam);
     * SmartDashboard.putNumber("Vision/align_cam0_age_ms", (now - (lastGoodTs.length > 0 ?
     * lastGoodTs[0] : -1)) * 1000.0); SmartDashboard.putNumber("Vision/align_cam1_age_ms", (now -
     * (lastGoodTs.length > 1 ? lastGoodTs[1] : -1)) * 1000.0);
     */
    putDataInDashboard();
  }

  public void putDataInDashboard() {
    SmartDashboard.putNumber("Limelight Target ID", getTargetID());
  }

  // ========= NUEVAS APIS RELIABLE PARA ALINEACIÓN =========

  /** Mejor target disponible (con TTL) entre las cámaras de alineación. */
  public Optional<PhotonTrackedTarget> getAlignmentTargetReliable() {
    final double now = Timer.getFPGATimestamp();

    PhotonTrackedTarget pick = null;
    int pickCam = -1;
    double bestAmb = Double.POSITIVE_INFINITY;
    double bestYawA = Double.POSITIVE_INFINITY;
    double bestDist = Double.POSITIVE_INFINITY;

    for (int camIdx : HighAltitudeConstants.ALIGNMENT_CAMERAS) {
      if (camIdx < 0 || camIdx >= (lastGoodTarget == null ? 0 : lastGoodTarget.length))
        continue;

      var t = lastGoodTarget[camIdx];
      double ts = lastGoodTs[camIdx];
      if (t == null)
        continue;
      if (now - ts > ALIGN_TTL_SEC)
        continue; // target viejo

      double amb = t.poseAmbiguity;
      double yawA = Math.abs(t.getYaw());
      double dist = t.bestCameraToTarget.getTranslation().getNorm();

      double ambScore = (amb < 0) ? 0.0 : amb;
      if (ambScore + EPS < bestAmb || (Math.abs(ambScore - bestAmb) < EPS && (yawA + EPS < bestYawA
          || (Math.abs(yawA - bestYawA) < EPS && dist + EPS < bestDist)))) {
        pick = t;
        pickCam = camIdx;
        bestAmb = ambScore;
        bestYawA = yawA;
        bestDist = dist;
      }
    }

    lastChosenCam = pickCam; // para debug
    return Optional.ofNullable(pick);
  }

  /** ID del mejor target para alinear (con TTL). Retorna -1 si no hay válido. */
  public int getAlignmentTargetIdReliable() {
    return getAlignmentTargetReliable().map(PhotonTrackedTarget::getFiducialId).orElse(-1);
  }

  /** Yaw (deg) del mejor target para alinear, NaN si no hay válido. */
  public double getAlignmentTargetYawReliable() {
    return getAlignmentTargetReliable().map(PhotonTrackedTarget::getYaw).orElse(Double.NaN);
  }

  /** Distancia (m) del mejor target para alinear, NaN si no hay válido. */
  public double getAlignmentTargetDistanceReliable() {
    return getAlignmentTargetReliable().map(t -> t.bestCameraToTarget.getTranslation().getNorm())
        .orElse(Double.NaN);
  }

  /** ¿Hay target “fresco” dentro del TTL para usar en alineación? */
  public boolean hasFreshAlignmentTarget() {
    return getAlignmentTargetReliable().isPresent();
  }
}
