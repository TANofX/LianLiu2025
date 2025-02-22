package frc.lib.vision;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Simple class that stores Camera along with the position(aka cameraToRobot)
 * information
 */
public class Camera {
    private final PhotonCamera camera;
    private final Transform3d position;
    private final String prefix;

    private final WelfordSD sdX = new WelfordSD();
    private final WelfordSD sdY = new WelfordSD();
    private final WelfordSD sdTheta = new WelfordSD();

    /**
     * Create a new Camera object
     * 
     * @param name The PhotonCamera name
     * @param pos The position of the camera
     */
    public Camera(String name, Transform3d pos) {
        prefix = "Vision/AprilTag/" + name;
        camera = new PhotonCamera(name);
        position = pos;
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
}
