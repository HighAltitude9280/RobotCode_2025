// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import org.photonvision.targeting.PhotonPipelineResult;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  PhotonCamera poseCamFront, poseCamBack, alignmentCam;

  PhotonPoseEstimator poseEstimatorFront, poseEstimatorBack;


  List<PhotonPipelineResult> alignmentResults;

  /** Creates a new vision. */
  public Vision() {
    // TODO : Set camera names
    poseCamFront = new PhotonCamera("ArducamFront");
    poseCamBack = new PhotonCamera("ArducamBack");

    alignmentCam = new PhotonCamera("limelight");

    // Translation 3d use this doc to know the position on your robot.
    // https://docs.wpilib.org/es/stable/docs/software/basic-programming/coordinate-system.html

    AprilTagFieldLayout fieldLayout;
    // Uncomment the try-catch and comment the next line to use a custom field
    // layout.
    /*
     * try {
     * fieldLayout = new AprilTagFieldLayout(
     * "/home/lvuser/deploy/vision/CustomAprilTagFieldLayout.json");
     * } catch (IOException e) {
     * 
     * e.printStackTrace();
     * }
     */

    fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    Transform3d camFront = new Transform3d(new Translation3d(0, 0, 0),
        new Rotation3d(0f, Math.toRadians(-10), Math.toRadians(190)));

    poseEstimatorFront = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        camFront);

    Transform3d camBack = new Transform3d(new Translation3d(0, 0, 0),
        new Rotation3d(0f, Math.toRadians(-10), Math.toRadians(190)));

    poseEstimatorBack = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        camBack);

  }

  public ArrayList<Optional<EstimatedRobotPose>> getEstimatedGlobalPose() 
  {
    Optional<EstimatedRobotPose> pose1 = Optional.empty();
    Optional<EstimatedRobotPose> pose2 = Optional.empty();

    for (var change : poseCamFront.getAllUnreadResults()) 
    {
      pose1 = poseEstimatorFront.update(change);
    }
    for (var change : poseCamBack.getAllUnreadResults()) 
    {
      pose1 = poseEstimatorBack.update(change);
    }

    var res = new ArrayList<Optional<EstimatedRobotPose>>();
    res.add(pose1);
    res.add(pose2);

    return res;
  }

  public boolean alignmentCamHasTargets() {
    return alignmentResults == null || alignmentResults.isEmpty()
        || alignmentResults.get(alignmentResults.size() - 1).hasTargets();
  }

  public double getTargetYaw(int id) {
    for (var target : alignmentResults.get(alignmentResults.size() - 1).getTargets()) {
      if (target.getFiducialId() == id) {
        return target.getYaw();
      }
    }
    return Double.NaN;
  }

  public double getTargetSize(int id) {
    for (var target : alignmentResults.get(alignmentResults.size() - 1).getTargets()) {
      if (target.getFiducialId() == id) {
        return target.getArea();
      }
    }
    return Double.NaN;
  }

  @Override
  public void periodic() {}
}