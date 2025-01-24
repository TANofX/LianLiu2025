package frc.lib.vision;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Transform3d;

/**
 * Simple class that stores Camera along with the position(aka cameraToRobot) information
 */
public class Camera {
    private PhotonCamera camera;
    private Transform3d position;
    public Camera(PhotonCamera cam, Transform3d pos) {
        camera = cam;
        position = pos;
    }
    public PhotonCamera getCamera() {
        return camera;
    }
    public Transform3d getPosition() {
        return position;
    }
}
