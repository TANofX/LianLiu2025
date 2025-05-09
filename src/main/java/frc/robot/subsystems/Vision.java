package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Optional;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        addCamera("port", new Transform3d(
                Units.inchesToMeters(-13.280346266), // Cad Z
                Units.inchesToMeters(11.580914897), // Cad X
                Units.inchesToMeters(8.177878478), // Cad Y
                new Rotation3d(0, (-15 * Math.PI) /180.0, (90 * Math.PI)/180.0)));
        // addCamera("starboard", new Transform3d(
        //         Units.inchesToMeters(-13.280346266), // Cad Z
        //         Units.inchesToMeters(-11.580914897), // Cad X
        //         Units.inchesToMeters(8.177878478), // Cad Y
        //         new Rotation3d(0, (-15 * Math.PI) /180.0, (-90 * Math.PI)/180.0)));
        addCamera("elevator", new Transform3d(
                Units.inchesToMeters(4.734322834),
                Units.inchesToMeters(6.233641834),
                Units.inchesToMeters(30.718469),
                new Rotation3d(0, Units.degreesToRadians(45.0), Units.degreesToRadians(90))));
        addCamera("tilty", new Transform3d(
            Units.inchesToMeters(7.776),
            Units.inchesToMeters(7.156),
            Units.inchesToMeters(30.895),
            new Rotation3d(0, Units.degreesToRadians(45.0), Units.degreesToRadians(60))));
    }

    @Override
    public void periodic() {
        // Result best = new Result();
        // for (Camera cam : cameras) {
        //     List<PhotonPipelineResult> tagResults = cam.getResults();
        //     for (PhotonPipelineResult result : tagResults) {
        //         Result r = processResult(result, cam);
        //         if(((!best.found) || r.found) && (r.ambiguity < best.ambiguity)) best = r;
        //     }
        // }
        // if(best.found) {
        //     if (best.ambiguity < 0.01) {
        //         RobotContainer.swerve.odometry.setVisionMeasurementStdDevs(best.cam.getSD());
        //         RobotContainer.swerve.odometry.addVisionMeasurement(best.robotPose.toPose2d(), best.imageCaptureTime);
        //     }
        // }
        for (Camera cam: cameras) {
            // Optional<EstimatedRobotPose> robotPose = cam.getPoseEstimate();
            cam.updateOdometry(RobotContainer.swerve.odometry);
            // if (robotPose.isPresent()) {
            //     RobotContainer.swerve.odometry.setVisionMeasurementStdDevs(VecBuilder.fill(0.1, 0.1, 0.05));
            //     RobotContainer.swerve.odometry.addVisionMeasurement(robotPose.get().estimatedPose.toPose2d(), robotPose.get().timestampSeconds);
            // }
        }
    }

    private Result processResult(PhotonPipelineResult result, Camera cam) {
        Transform3d cameraToRobot = cam.getPosition();
        
        SmartDashboard.putBoolean(cam.prefix+"/tagPresent", result.hasTargets());

        if (result.hasTargets()) {
            PhotonTrackedTarget passedTarget = result.getBestTarget();
            Optional<Pose3d> tagPose = Constants.apriltagLayout.getTagPose(passedTarget.getFiducialId());
            

            if (tagPose.isPresent()) {
                SmartDashboard.putNumberArray(cam.prefix+"/cameraPose", new double[] {
                    cam.getPosition().getX(), cam.getPosition().getY(), cam.getPosition().getZ(), 
                    cam.getPosition().getRotation().getX(), cam.getPosition().getY(), cam.getPosition().getZ()
                });
                var imageCaptureTime = result.getTimestampSeconds();
                Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
                        passedTarget.getBestCameraToTarget(),
                        tagPose.get(),
                        cameraToRobot);

                SmartDashboard.putNumberArray(cam.prefix+"/RobotPose3D", new double[] {
                    robotPose.getX(), robotPose.getY(), robotPose.getZ(),
                    robotPose.getRotation().getX(), robotPose.getRotation().getY(), robotPose.getRotation().getZ()
                });
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
                Pose2d pose = robotPose.toPose2d();
                SmartDashboard.putNumberArray(cam.prefix+"/robotPose", new double[] {
                    pose.getX(), pose.getY(), pose.getRotation().getDegrees()
                });
                SmartDashboard.putNumberArray(cam.prefix+"/sd", cam.getSD().getData());
                return new Result(cam, robotPose, imageCaptureTime, passedTarget.getPoseAmbiguity());
            }
        }
        return new Result();
    }

    public void addCamera(String cameraName, Transform3d pos) {
        cameras.add(new Camera(cameraName, pos));
    }

    @Override
    public Command systemCheckCommand() {
        return Commands.none();
    }
}

class Result {
    public final Camera cam;
    public final Pose3d robotPose;
    public final double imageCaptureTime;
    public final double ambiguity;
    public final boolean found;

    public Result() {
        found = false;
        cam = null;
        robotPose = null;
        ambiguity = 2;
        imageCaptureTime = 0;
    }
    public Result(Camera c, Pose3d r, double i, double a) {
        cam = c;
        robotPose = r;
        imageCaptureTime = i;
        ambiguity = a;
        found = true;
    }
}