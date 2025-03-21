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

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HighAltitudeConstants;

public class Vision extends SubsystemBase {

  ArrayList<PhotonCamera> cams;
  ArrayList<PhotonPoseEstimator> poseEstimators;
  ArrayList<List<PhotonPipelineResult>> results;

  /** Creates a new vision. */
  public Vision() 
  {
    cams = new ArrayList<>();
    poseEstimators = new ArrayList<>();
    results = new ArrayList<>();

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

    for(var name : HighAltitudeConstants.CAMERA_NAMES)
    {
      cams.add(new PhotonCamera(name));
    }
    for(var robotToCamera : HighAltitudeConstants.CAMERA_POSITIONS)
    {
      poseEstimators.add(new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
        robotToCamera));
    }
  }

  public ArrayList<EstimatedRobotPose> getEstimatedRobotPoses()
  {
    var poses = new ArrayList<EstimatedRobotPose>();

    if(results.isEmpty()) return poses;

    for(int i = 0 ; i < cams.size() ; i ++)
    {
      Optional<EstimatedRobotPose> pose = Optional.empty();

      for(var result : results.get(i))
      {
        if(!result.hasTargets()) continue;

        var dist = result.getBestTarget().bestCameraToTarget.getTranslation().getNorm();
        var ambig = result.getBestTarget().poseAmbiguity;

        if(dist > HighAltitudeConstants.VISION_POSE_ESTIMATOR_MAX_DISTANCE ||
          ambig > HighAltitudeConstants.VISION_POSE_ESTIMATOR_MAX_AMBIGUITY) continue;

        pose = poseEstimators.get(i).update(result);
      }
      if(pose.isPresent()) poses.add(pose.get());
    }
    return poses;
  }

  public boolean alignmentCamHasTargets() 
  {
    if(results.isEmpty()) return false;
    for(int i : HighAltitudeConstants.ALIGNMENT_CAMERAS)
    {
      if(!results.get(i).isEmpty() && results.get(i).get(results.get(i).size() -1).hasTargets()) 
        return true;
    }
    return false;
  }

  public int getTargetID() 
  {
    if (results.isEmpty()) return-1;
    else
    {
      for(int i : HighAltitudeConstants.ALIGNMENT_CAMERAS)
      {
        if(!results.get(i).isEmpty())
        {
          var target = results.get(i).get(results.get(i).size() -1).getBestTarget();
          if(target == null) continue;
          else return target.getFiducialId();
        }
      }
    }
    return -1;
  }
  /** 
  public double getTargetYaw(int id) {
    if (alignmentResults == null || alignmentResults.isEmpty())
      return Double.NaN;
    else
      for (var target : alignmentResults.get(alignmentResults.size() - 1).getTargets()) {
        if (target.getFiducialId() == id) {
          return target.getYaw();
        }
      }

    return Double.NaN;
  }

  public double getTargetYaw() {
    if (alignmentResults == null || alignmentResults.isEmpty())
      return Double.NaN;
    else {
      var target = alignmentResults.get(alignmentResults.size() - 1).getBestTarget();
      if (target == null)
        return Double.NaN;

      return target.yaw;
    }
  }

  public double getTargetSize(int id) {
    if (alignmentResults == null || alignmentResults.isEmpty())
      return Double.NaN;
    else
      for (var target : alignmentResults.get(alignmentResults.size() - 1).getTargets()) {
        if (target.getFiducialId() == id) {
          return target.getArea();
        }
      }
    return Double.NaN;
  }

  public double getTargetSize() {
    if (alignmentResults == null || alignmentResults.isEmpty())
      return Double.NaN;
    else {
      var target = alignmentResults.get(alignmentResults.size() - 1).getBestTarget();
      if (target == null)
        return Double.NaN;
      return target.area;
    }
  }
    */

  @Override
  public void periodic() 
  {
    results.clear();
    for(var cam : cams)
    {
      results.add(cam.getAllUnreadResults());
    }
  }

  public void putDataInDashboard() {
    SmartDashboard.putNumber("Limelight Target ID", getTargetID());
  }
}