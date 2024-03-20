package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.concurrent.atomic.AtomicReference;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AprilTag extends SubsystemBase{
    private final PhotonCamera photonCamera;
    private final PhotonPoseEstimator poseEstimator;
    private final AtomicReference<EstimatedRobotPose> atomicEstimatedAtomicReference = new AtomicReference<EstimatedRobotPose>();
    private PhotonTrackedTarget note;
    private PhotonPipelineResult photonResults;
    
    public AprilTag(PhotonCamera camera, Transform3d robotToCamera){
        this.photonCamera = camera;
        if(photonCamera != null){
         PhotonPoseEstimator Estimator = new PhotonPoseEstimator(AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
         photonCamera, robotToCamera);
         this.poseEstimator = Estimator;
        }
    }

    @Override
    public void run(){
        if(poseEstimator != null && photonCamera != null){
            PhotonPipelineResult photonResults = photonCamera.getLatestResult();
            if(photonResults.hasTargets() && photonResults.targets.get(0).getPoseAmbiguity() < 0.2){
                poseEstimator.update(photonResults).ifPresent(estimatedRobotPose ->{
                    Pose3d estimatedPose = estimatedRobotPose.estimatedPose;
                    if(estimatedPose.getX() > 0.0 && estimatedPose.getX() <= 16.51 && estimatedPose.getY() > 0.0 && estimatedPose.getY() <= 8.211 && Math.abs(estimatedPose.getZ()) <= 0.15){
                        atomicEstimatedAtomicReference.set(estimatedRobotPose);                        
                    }
                });
            }
        }
    }

    public EstimatedRobotPose grabLatestEstimatedPose(){
        return atomicEstimatedAtomicReference.get();
    }
    
}
