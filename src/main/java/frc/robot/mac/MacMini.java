package frc.robot.mac;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.UnknownHostException;
import java.nio.ByteBuffer;
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
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;


public class MacMini {
        // Camera info
        private record CameraInfo(PhotonCamera camera, PhotonPoseEstimator poseEstimator) {}; 
        private record VisionInfo(PhotonPipelineResult result, EstimatedRobotPose pose) {};

        // nt
        NetworkTableInstance photonNT = NetworkTableInstance.create();

        // Cameras
        CameraInfo[] cameras;

        DatagramSocket socket;

        public MacMini() {
            try {
                socket = new DatagramSocket();
            } catch (Exception e) {}

            photonNT.setServer("localhost", 5810);
            photonNT.startClient4("mac-photon-client");

            cameras = new CameraInfo[VisionConstants.CAMERA_CONSTANTS.length];
            
            // Create the cameras
            for (int i = 0; i < VisionConstants.CAMERA_CONSTANTS.length; i++) {
                log("Creating " + VisionConstants.CAMERA_CONSTANTS[i].name() + " info");
                AprilTagFieldLayout fieldLayout;

                try {
                    // Get the path to the field from the deploy directory
                    Path fieldPath = Path.of(
                        System.getProperty("user.home"),
                        "Users",
                        "lightning",
                        "field_layout.json"
                    );

                    fieldLayout = new AprilTagFieldLayout(fieldPath);
                } catch (Exception e) {
                    // Just use the default field if we can't get it
                    log("Can't load field resource-- using default field");
                    fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
                }

                // Get the pose estimator
                PhotonPoseEstimator poseEstimator =
                        new PhotonPoseEstimator(
                                fieldLayout,
                                VisionConstants.CAMERA_CONSTANTS[i].offset()
                        );
                    
                // Create the camera using the name from our constant
                PhotonCamera camera = new PhotonCamera(photonNT, VisionConstants.CAMERA_CONSTANTS[i].name());

                // Create the camera
                cameras[i] = new CameraInfo(camera, poseEstimator);
            }
        }

        public void run() {
            // System.out.println("Something is running");

            // StructPublisher<Pose2d> posePublisher = nt.getTable("Mac").getStructTopic("estimated_pose", Pose2d.struct).publish();
            // DoublePublisher ambiguityPublisher = nt.getTable("Mac").getDoubleTopic("pose_ambiguity").publish();
            // DoublePublisher timestampPublisher = nt.getTable("Mac").getDoubleTopic("pose_timestamp").publish();
            // IntegerPublisher resultCounterPublisher = nt.getTable("Mac").getIntegerTopic("result_counter").publish();

            int counter = 0;

            while (true) {
                VisionInfo info = getEstimatedPose();

                if (info.pose != null && info.result != null) {
                    Pose2d poseToPublish = info.pose().estimatedPose.toPose2d();
                    double ambiguity = info.result().getBestTarget().poseAmbiguity;
                    double timestamp = info.result().getTimestampSeconds();

                    counter++;
                    try {
                         DatagramPacket packet = getBinaryPacket(poseToPublish, ambiguity, timestamp, counter);
                         socket.send(packet);
                        System.out.println("PACKET SENT");
                         
                    } catch (Exception e) {
                        log("Failed to send packet" + e);
                        
                    }

                    

                    // posePublisher.set(poseToPublish);
                    // ambiguityPublisher.set(ambiguity);
                    // timestampPublisher.set(timestamp);


                    // resultCounterPublisher.set(counter);

                }
                
                try {
                    Thread.sleep(1);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }

        public void shutdown() {
            photonNT.close();
        }

        private VisionInfo getEstimatedPose() {
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
            return bestPose == null ? new VisionInfo(null, null) : bestPose;
        }

        private VisionInfo getVisionPose(CameraInfo cameraInfo) {
            PhotonCamera camera = cameraInfo.camera;

            List<PhotonPipelineResult> results = camera.getAllUnreadResults();
            // log("NUMBER OF RESULTS: " + results.size());
            // log("Camera name " + camera.getName());
            // If theres no results just skip this iteration
            if (results.isEmpty()) {
                // log(cameraInfo.camera.getName() + "'s Result is null");
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
                
                // Add the vision measurment
                return new VisionInfo(useableResult, pose);
            } else {
                // Get the estimated position
                poseOpt = cameraInfo.poseEstimator().estimateLowestAmbiguityPose(useableResult);
                
                if (poseOpt.isPresent()) {
                    // The pose
                    EstimatedRobotPose pose = poseOpt.get();

                    // Add the vision measurment
                    return new VisionInfo(useableResult, pose);
                }
            }

            // We have no pose if were here
            log("No pose");
            return null;
        }

        private void log(String message) {
            // System.out.println("[PHOTON VISION]" + message);
        }

        private DatagramPacket getBinaryPacket(Pose2d pose, double ambiguity, double timestamp, double counter) throws IllegalArgumentException, UnknownHostException {// 8 + 8 + 8 + 8 + 8 bytes
            ByteBuffer buffer = ByteBuffer.allocate(48);

            buffer.putDouble(pose.getX());
            buffer.putDouble(pose.getY());
            buffer.putDouble(pose.getRotation().getRadians());
            buffer.putDouble(ambiguity);
            buffer.putDouble(timestamp);
            buffer.putDouble(counter);

            // // Pack pose X
            // long xBits = Double.doubleToLongBits(pose.getX());
            // for (int i = 0; i < 8; i++) {
            //     data[i] = (byte) ((xBits >> (8 * i)) & 0xFF);
            // }
            
            // // Pack pose Y
            // long yBits = Double.doubleToLongBits(pose.getY());
            // for (int i = 0; i < 8; i++) {
            //     data[8 + i] = (byte) ((yBits >> (8 * i)) & 0xFF);
            // }
            
            // // Pack rotation (in radians)
            // long rotationBits = Double.doubleToLongBits(pose.getRotation().getRadians());
            // for (int i = 0; i < 8; i++) {
            //     data[16 + i] = (byte) ((rotationBits >> (8 * i)) & 0xFF);
            // }
            
            // // Pack ambiguity
            // long ambiguityBits = Double.doubleToLongBits(ambiguity);
            // for (int i = 0; i < 8; i++) {
            //     data[24 + i] = (byte) ((ambiguityBits >> (8 * i)) & 0xFF);
            // }
            
            // // Pack timestamp
            // long timestampBits = Double.doubleToLongBits(timestamp);
            // for (int i = 0; i < 8; i++) {
            //     data[32 + i] = (byte) ((timestampBits >> (8 * i)) & 0xFF);
            // }

            // // Pack timestamp
            // long counterBits = Double.doubleToLongBits(counter);
            // for (int i = 0; i < 8; i++) {
            //     data[40 + i] = (byte) ((counterBits >> (8 * i)) & 0xFF);
            // }
            byte[] data = buffer.array();
            return new DatagramPacket(data, data.length, InetAddress.getByName("10.8.62.2"), 12345);
        }
    }