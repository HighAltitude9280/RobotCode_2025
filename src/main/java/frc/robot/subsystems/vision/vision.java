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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class vision extends SubsystemBase {
  PhotonCamera tagsCameraFront, tagsCameraBack;
  PhotonPipelineResult resultFront, resultBack;

  PhotonPoseEstimator poseEstimatorFront, poseEstimatorBack;

  private ArrayList<Optional<EstimatedRobotPose>> estimatedPoseFront, estimatedPoseBack; // TODO: fix this

  private Optional<EstimatedRobotPose> pose1;
  private Optional<EstimatedRobotPose> pose2;

  /** Creates a new vision. */
  public vision() {
    tagsCameraFront = new PhotonCamera("ArducamFront");
    tagsCameraBack = new PhotonCamera("ArducamBack");

    estimatedPoseFront = new ArrayList<>();
    estimatedPoseBack = new ArrayList<>();

    AprilTagFieldLayout fieldLayout;
    try {
      // Translation 3d use this doc to know the position on your robot.
      // https://docs.wpilib.org/es/stable/docs/software/basic-programming/coordinate-system.html
      fieldLayout = new AprilTagFieldLayout(
          "/home/lvuser/deploy/vision/CustomAprilTagFieldLayout.json");

      Transform3d camFront = new Transform3d(new Translation3d(0, 0, 0),
          new Rotation3d(0f, Math.toRadians(-10), Math.toRadians(190)));
      poseEstimatorFront = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          camFront);

      Transform3d camBack = new Transform3d(new Translation3d(0, 0, 0),
          new Rotation3d(0f, Math.toRadians(-10), Math.toRadians(190)));
      poseEstimatorBack = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          camBack);

    } catch (IOException e) {

      e.printStackTrace();
    }

    // Initialize pose1 and pose2 after poseEstimators are created
    pose1 = poseEstimatorBack.update(resultBack);
    pose2 = poseEstimatorFront.update(resultFront);

    resultBack = tagsCameraBack.getLatestResult();
    resultFront = tagsCameraFront.getLatestResult();
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

    poseEstimatorFront.setReferencePose(prevEstimatedRobotPose); //TODO: creo q falta uno para back
    ArrayList<Optional<EstimatedRobotPose>> result = new ArrayList<>();
    result.add(pose1);
    result.add(pose2);
    return result;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pose1 = poseEstimatorBack.update(resultBack);
    pose2 = poseEstimatorFront.update(resultFront);
    getEstimatedPosition();
  }
}