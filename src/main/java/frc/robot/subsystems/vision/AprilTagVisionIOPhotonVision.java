// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.FieldConstants;
import frc.robot.util.VisionHelpers.PoseEstimate;
import java.util.ArrayList;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

// This class implements the AprilTagVisionIO interface and uses the PhotonVision library
// to process vision data from a camera and estimate the robot's pose relative to AprilTags on the
// field.
public class AprilTagVisionIOPhotonVision implements AprilTagVisionIO {
  // The camera from which the vision data is received
  private final PhotonCamera camera;
  // The pose estimator used to estimate the robot's pose
  private final PhotonPoseEstimator photonEstimator;

  // The timestamp of the last pose estimate
  private double lastEstTimestamp = 0;

  // Constructor for the AprilTagVisionIOPhotonVision class
  public AprilTagVisionIOPhotonVision(String identifier, Transform3d robotToCamera) {
    // Initialize the camera and pose estimator
    camera = new PhotonCamera(identifier);
    photonEstimator =
        new PhotonPoseEstimator(
            FieldConstants.aprilTags,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            camera,
            robotToCamera);
    // Set the fallback strategy for the pose estimator
    photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  // Method to update the inputs for AprilTag vision
  @Override
  public void updateInputs(AprilTagVisionIOInputs inputs) {
    // Get the latest result from the camera
    PhotonPipelineResult results = camera.getLatestResult();
    // Create a list to store pose estimates
    ArrayList<PoseEstimate> poseEstimates = new ArrayList<>();
    // Get the timestamp of the result
    double timestamp = results.getTimestampSeconds();
    // Get the alliance color
    Optional<Alliance> allianceOptional = DriverStation.getAlliance();
    // If there are targets and the alliance color is known
    if (!results.targets.isEmpty() && allianceOptional.isPresent()) {
      // Get the latency of the result
      double latencyMS = results.getLatencyMillis();
      // Declare a variable to store the pose estimation
      Pose3d poseEstimation;
      // Get the estimated global pose
      Optional<EstimatedRobotPose> estimatedPose = getEstimatedGlobalPose();
      // If the estimated pose is not present, return
      if (estimatedPose.isEmpty()) {
        return;
      }
      // Get the estimated pose
      poseEstimation = estimatedPose.get().estimatedPose;
      // Initialize a variable to store the average tag distance
      double averageTagDistance = 0.0;
      // Adjust the timestamp for latency
      timestamp -= (latencyMS / 1e3);
      // Create an array to store the tag IDs
      int[] tagIDs = new int[results.targets.size()];
      // For each target
      for (int i = 0; i < results.targets.size(); i++) {
        // Get the tag ID
        tagIDs[i] = results.targets.get(i).getFiducialId();
        // Get the pose of the tag
        var tagPose = photonEstimator.getFieldTags().getTagPose(tagIDs[i]);
        // If the tag pose is not present, continue to the next target
        if (tagPose.isEmpty()) {
          continue;
        }
        // Add the distance to the tag to the average tag distance
        averageTagDistance +=
            tagPose
                .get()
                .toPose2d()
                .getTranslation()
                .getDistance(poseEstimation.getTranslation().toTranslation2d());
      }
      // Calculate the average tag distance
      averageTagDistance /= tagIDs.length;
      // Add the pose estimate to the list
      poseEstimates.add(new PoseEstimate(poseEstimation, timestamp, averageTagDistance, tagIDs));
      // Set the pose estimates in the inputs
      inputs.poseEstimates = poseEstimates;
    }
  }

  // Method to update the PhotonPoseEstimator and return the estimated global pose
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    // Update the pose estimator
    var visionEst = photonEstimator.update();
    // Get the timestamp of the latest result
    double latestTimestamp = camera.getLatestResult().getTimestampSeconds();
    // Check if there is a new result
    boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
    // If there is a new result, update the last estimate timestamp
    if (newResult) lastEstTimestamp = latestTimestamp;
    // Return the estimated pose
    return visionEst;
  }
}
