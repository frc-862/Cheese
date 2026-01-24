package frc.robot.mac;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;

public class MacMini {
        // Camera info
        private record CameraInfo(PhotonCamera camera, PhotonPoseEstimator poseEstimator) {}; 
        private record VisionInfo(PhotonPipelineResult result, EstimatedRobotPose pose) {};

        // Cameras
        CameraInfo[] cameras;

        public  MacMini() {
            System.out.println("[PHOTON VISION] Starting Mac Mini Vision Processor");

            cameras = new CameraInfo[VisionConstants.CAMERA_CONSTANTS.length];
            
            // Create the cameras
            for (int i = 0; i < VisionConstants.CAMERA_CONSTANTS.length; i++) {
                System.out.println("[PHOTON VISION] Creating " + VisionConstants.CAMERA_CONSTANTS[i].name() + " info");
                AprilTagFieldLayout fieldLayout;

                try {
                    // Get the path to the field from the deploy directory
                    Path fieldPath = Path.of(
                        System.getProperty("user.home"),
                        "photonvision",
                        "field_layout.json"
                    );

                    fieldLayout = new AprilTagFieldLayout(fieldPath);
                } catch (Exception e) {
                    // Just use the default field if we can't get it
                    log("Can't load field resource-- using default field");
                    fieldLayout = VisionConstants.REBUILT_FIELD;
                }

                // Get the pose estimator
                PhotonPoseEstimator poseEstimator =
                        new PhotonPoseEstimator(
                                VisionConstants.REBUILT_FIELD,
                                VisionConstants.CAMERA_CONSTANTS[i].offset()
                        );
                    
                // Create the camera using the name from our constant
                PhotonCamera camera = new PhotonCamera(VisionConstants.CAMERA_CONSTANTS[i].name());

                // Create the camera
                cameras[i] = new CameraInfo(camera, poseEstimator);
                log(VisionConstants.CAMERA_CONSTANTS[i].name() + " info created sucessfully");

            }
        }

        public void run() {
            NetworkTableInstance nt = NetworkTableInstance.getDefault();
            StructPublisher<Pose2d> posePublisher = nt.getTable("Mac").getStructTopic("estimated_pose", Pose2d.struct).publish();
            DoublePublisher ambiguityPublisher = nt.getTable("Mac").getDoubleTopic("pose_ambiguity").publish();
            DoublePublisher timestampPublisher = nt.getTable("Mac").getDoubleTopic("pose_timestamp").publish();

            while (true) {
                posePublisher.set(getEstimatedPose().pose().estimatedPose.toPose2d());

                ambiguityPublisher.set(getEstimatedPose().result()==null ? 1 : getEstimatedPose().result().getBestTarget().poseAmbiguity);
                timestampPublisher.set(getEstimatedPose().result()==null ? -1 : getEstimatedPose().result().getTimestampSeconds());
            }
        }

        public VisionInfo getEstimatedPose() {
            if (cameras == null || cameras.length == 0) {
                log("No cameras configured");
                return null;
            }

            try {
                VisionInfo[] poses = new VisionInfo[VisionConstants.CAMERA_CONSTANTS.length];
                
                for (int i = 0; i < VisionConstants.CAMERA_CONSTANTS.length; i++) {
                    poses[i] = getVisionPose(cameras[i]);
                }

                return getBestPose(poses);
            } catch (Exception e) {
                log("Failed to get pose");
                e.printStackTrace();
                return null;
            }
        }

        // Gets the latest result from multiple results
        private PhotonPipelineResult getLatestResult(List<PhotonPipelineResult> results) {
            int latestResultIndex = 0;

            for (int i = 0; i < results.size(); i++) {
                if (results.get(i).getTimestampSeconds() > results.get(latestResultIndex).getTimestampSeconds()) {
                    latestResultIndex = i;
                }
            }

            PhotonPipelineResult latestResult = results.get(latestResultIndex);

            return latestResult;
        }

        // This will get the best pose
        private VisionInfo getBestPose(VisionInfo[] visionInfos) {
            VisionInfo bestPose = null;

            for (VisionInfo info : visionInfos) {
                if (info == null) {
                    continue;
                }
                
                if (bestPose == null) {
                    bestPose = info;
                    continue;
                }

                if (bestPose.result().getBestTarget().poseAmbiguity > info.result().getBestTarget().poseAmbiguity) {
                    bestPose = info;
                }
            }
            log("Got best pose");
            return bestPose == null ? new VisionInfo(null, null) : bestPose;
        }

        private VisionInfo getVisionPose(CameraInfo cameraInfo) {
            PhotonCamera camera = cameraInfo.camera;

            List<PhotonPipelineResult> results = camera.getAllUnreadResults();

            log("Results recieved" + results.size());
            
            // If theres no results just skip this iteration
            if (results.isEmpty()) {
                log(cameraInfo.camera.getName() + "'s Result is null");
                return null;
            }
            
            // Get the latest result of all thme
            PhotonPipelineResult latestResult = getLatestResult(results);

            // Filter out the targets
            List<PhotonTrackedTarget> filteredTargets = new ArrayList<>(latestResult.getTargets());
            filteredTargets.removeIf((tag) -> VisionConstants.TAG_IGNORE_LIST.contains((short) tag.getFiducialId()));

            // Scrap it if the new result has no target
            if (filteredTargets.isEmpty()) {
                return null;
            }

            PhotonPipelineResult useableResult = latestResult;

            // Create a new result to use -- Using the same metadata as the original latest result
            if (!latestResult.getTargets().stream().allMatch((target) -> filteredTargets.contains(target))) {
                useableResult = new PhotonPipelineResult(
                    latestResult.metadata,
                    filteredTargets,
                    Optional.empty()
                );
            }

            // If pose ambiguity is to high well scrap the result
            boolean highPoseAmbiguity = latestResult.getBestTarget().getPoseAmbiguity() > VisionConstants.POSE_AMBIGUITY_TOLERANCE;

            // If the best tag's distance is too far than scrap the result
            double bestDistance = useableResult.getBestTarget().getBestCameraToTarget().getTranslation().getNorm();
            boolean highDistance = bestDistance > VisionConstants.TAG_DISTANCE_TOLERANCE;

            // If we have high ambiguity or high distance then just return back a null/"empty" value
            if (highPoseAmbiguity || highDistance) {
                return null;
            }
            
            // Get the estimated position
            Optional<EstimatedRobotPose> poseOpt = cameraInfo.poseEstimator().estimateCoprocMultiTagPose(useableResult);

            // If the estimated position is there run this code
            if (poseOpt.isPresent()) {
                // The pose
                EstimatedRobotPose pose = poseOpt.get();
                
                log("Used multitag result");

                // Add the vision measurment
                return new VisionInfo(useableResult, pose);
            } else {
                // Get the estimated position
                poseOpt = cameraInfo.poseEstimator().estimateLowestAmbiguityPose(useableResult);
                
                if (poseOpt.isPresent()) {
                    // The pose
                    EstimatedRobotPose pose = poseOpt.get();

                    log("Used singletag result");

                    // Add the vision measurment
                    return new VisionInfo(useableResult, pose);
                }
            }

            // We have no pose if were here
            log("No pose");
            return null;
        }

        private void log(String message) {
            System.out.println("[PHOTON VISION]" + message);
        }
    }