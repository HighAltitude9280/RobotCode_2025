// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import org.photonvision.targeting.PhotonPipelineResult;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  PhotonCamera poseCamFront, poseCamBack, alignmentCam;
  PhotonPipelineResult resultFront, resultBack;

  PhotonPoseEstimator poseEstimatorFront, poseEstimatorBack;

  private Optional<EstimatedRobotPose> pose1;
  private Optional<EstimatedRobotPose> pose2;

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

    // Initialize pose1 and pose2 after poseEstimators are created
    pose1 = poseEstimatorBack.update(resultBack);
    pose2 = poseEstimatorFront.update(resultFront);

    resultBack = poseCamBack.getLatestResult();
    resultFront = poseCamFront.getLatestResult();
  }

  public ArrayList<Optional<EstimatedRobotPose>> getEstimatedPosition() {
    ArrayList<Optional<EstimatedRobotPose>> result = new ArrayList<>();
    result.add(pose1);
    result.add(pose2);
    return result;
  }

  public ArrayList<Optional<EstimatedRobotPose>> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    // Check if prevEstimatedRobotPose is null
    if (prevEstimatedRobotPose == null) {
      // Handle the null case, e.g., log a warning or return an empty list
      return new ArrayList<>(List.of(pose1, pose2)); // Return current poses without updating
    }

    poseEstimatorFront.setReferencePose(prevEstimatedRobotPose);
    poseEstimatorBack.setReferencePose(prevEstimatedRobotPose);
    ArrayList<Optional<EstimatedRobotPose>> result = new ArrayList<>();
    result.add(pose1);
    result.add(pose2);
    return result;
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
  public void periodic() {
    // This method will be called once per scheduler run
    resultFront = poseCamFront.getLatestResult();
    resultBack = poseCamBack.getLatestResult();

    pose1 = poseEstimatorBack.update(resultBack);
    pose2 = poseEstimatorFront.update(resultFront);
    alignmentResults = alignmentCam.getAllUnreadResults();
  }
}