package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import static edu.wpi.first.units.Units.Inch;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.lib.vision.Camera;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.util.RobotPoseLookup;

public final class Vision extends AdvancedSubsystem {
    private final ArrayList<Camera> cameras = new ArrayList<>();
    private final Field2d aprilField = new Field2d();

    private boolean wasStopped = true;
    private boolean isStopped = true;

    public Vision() {
        addCamera("swerveL", new Transform3d(
                Inch.of(13.280346266), // Cad Z
                Inch.of(-11.580914897), // Cad X
                Inch.of(8.177878478), // Cad Y
                new Rotation3d(0, -15, -90)));
        addCamera("swerveR", new Transform3d(
                Inch.of(13.280346266), // Cad Z
                Inch.of(11.580914897), // Cad X
                Inch.of(8.177878478), // Cad Y
                new Rotation3d(0, -15, 90)));
    }

    @Override
    public void periodic() {
        for (Camera cam : cameras) {
            List<PhotonPipelineResult> results = cam.getResults();
            for (PhotonPipelineResult result : results)
                processResult(result, cam);
        }
    }

    private void processResult(PhotonPipelineResult result, Camera cam) {
        Transform3d cameraToRobot = cam.getPosition();
        if (result.hasTargets()) {
            PhotonTrackedTarget passedTarget = result.getBestTarget();
            Optional<Pose3d> tagPose = Constants.apriltagLayout.getTagPose(passedTarget.getFiducialId());

            if (tagPose.isPresent()) {
                var imageCaptureTime = result.getTimestampSeconds();
                Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
                        passedTarget.getBestCameraToTarget(),
                        tagPose.get(),
                        cameraToRobot);

                RobotPoseLookup<Pose3d> AprilTagLookup = new RobotPoseLookup<>();
                AprilTagLookup.addPose(robotPose);
                aprilField.setRobotPose(robotPose.toPose2d());
                ChassisSpeeds speeds = RobotContainer.swerve.getCurrentSpeeds();
                // Very rough estimate
                isStopped = speeds.omegaRadiansPerSecond / 10 + speeds.vxMetersPerSecond
                        + speeds.vyMetersPerSecond < 0.3;
                if (!wasStopped && isStopped) {
                    // Reset SD once after the robot stops
                    cam.resetSD();
                } else if (isStopped) {
                    cam.addSD(robotPose);
                }
                wasStopped = isStopped;
                if (passedTarget.getPoseAmbiguity() < 0.10) {
                    RobotContainer.swerve.odometry.setVisionMeasurementStdDevs(cam.getSD());
                    RobotContainer.swerve.odometry.addVisionMeasurement(robotPose.toPose2d(), imageCaptureTime);
                }
            }
        }
    }

    public void addCamera(String cameraName, Transform3d pos) {
        cameras.add(new Camera(cameraName, pos));
    }

    @Override
    public Command systemCheckCommand() {
        return Commands.none();
    }
}
