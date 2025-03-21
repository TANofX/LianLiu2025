package frc.lib.vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * Simple class that stores Camera along with the position(aka cameraToRobot)
 * information
 */
public class Camera {
    private final PhotonCamera camera;
    private final Transform3d position;
    public final String prefix;
    private final PhotonPoseEstimator poseEstimator;

    private final WelfordSD sdX = new WelfordSD();
    private final WelfordSD sdY = new WelfordSD();
    private final WelfordSD sdTheta = new WelfordSD();

    StructPublisher<Pose3d> m_pose_estimate;

    /**
     * Create a new Camera object
     * 
     * @param name The PhotonCamera name
     * @param pos  The position of the camera
     */
    public Camera(String name, Transform3d pos) {
        prefix = "Vision/AprilTag/" + name;
        camera = new PhotonCamera(name);
        position = pos; // why documentation no match :(
        poseEstimator = new PhotonPoseEstimator(Constants.apriltagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                position);
        m_pose_estimate = NetworkTableInstance.getDefault().getStructTopic(prefix + "/estimatedPose", Pose3d.struct)
                .publish();
    }

    /**
     * Get the PhotonCamera object
     * 
     * @return The PhotonCamera object
     */
    public PhotonCamera getCamera() {
        return camera;
    }

    /**
     * Get the position of the camera
     * 
     * @return The position of the camera
     */
    public Transform3d getPosition() {
        return position;
    }

    /**
     * Reset the standard deviation calculations
     */
    public void resetSD() {
        sdX.reset();
        sdY.reset();
        sdTheta.reset();
    }

    /**
     * Add a new robot pose to the standard deviation calculations
     * 
     * @param robotPose The new robot pose
     */
    public void addSD(Pose3d robotPose) {
        Pose2d pose2d = robotPose.toPose2d();
        sdX.addValue(pose2d.getX());
        sdY.addValue(pose2d.getY());
        sdTheta.addValue(pose2d.getRotation().getDegrees());
    }

    /**
     * Get the standard deviations of the robot pose
     * 
     * @return A matrix containing the standard deviations of the robot pose
     */
    public Matrix<N3, N1> getSD() {
        return VecBuilder.fill(
                sdX.getStandardDeviation(),
                sdY.getStandardDeviation(),
                sdTheta.getStandardDeviation());
    }

    public List<PhotonPipelineResult> getResults() {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        SmartDashboard.putBoolean(prefix + "/online", camera.isConnected());
        SmartDashboard.putNumber(prefix + "/results", results.size());
        return results;
    }

    public void updateOdometry(SwerveDrivePoseEstimator odometry) {

        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
            Optional<EstimatedRobotPose> output = poseEstimator.update(result);
            if (output.isPresent()) {
                m_pose_estimate.set(output.get().estimatedPose);
                double ambiguity = result.getBestTarget().getPoseAmbiguity();
                double tagDistance = result.getBestTarget().bestCameraToTarget.getTranslation().getNorm();

                if ((ambiguity < 0.05) && (tagDistance < 3.0)) {
                    odometry.addVisionMeasurement(output.get().estimatedPose.toPose2d(), output.get().timestampSeconds);
                }
            }
        }
    }
}
