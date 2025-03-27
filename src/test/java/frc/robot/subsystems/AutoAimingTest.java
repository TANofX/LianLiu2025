package frc.robot.subsystems;
import java.util.Arrays;
import java.util.Optional;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
import org.junit.jupiter.api.Test;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.util.RobotMechanism;

/** Add your docs here. */
public class AutoAimingTest {
    @Test
    public void testZeros() throws Exception {
        Rotation2d r = new Rotation2d();

        Pose2d robotPose = new Pose2d(3.95,0, r);
        RobotMechanism robotMechanism = new RobotMechanism(() -> robotPose);
        AutoAiming autoAimer = new AutoAiming(() -> robotMechanism.getFieldPositionOfCoralHandler(), () -> Optional.empty());
        
        Rotation2d hAngle = autoAimer.horizontalRotationToCoral();

        System.out.println();
        System.out.println("Robot Pose === " + robotPose.getTranslation());
        System.out.println("end of coral handler === " + robotMechanism.getFieldPositionOfCoralHandler());
        System.out.println("translation needed === " + hAngle);
        System.out.println("horizontal angle === " + (hAngle));

    }

    @Test
    public void testPosts() throws Exception {
        for (Translation2d trans: Constants.CoralPlacement.coordinatesCoral) {
            Pose2d pose = new Pose2d(trans, new Rotation2d());
            AprilTag nearestTag = getNearestReefAprilTag(pose);
        
            Transform2d transform = new Transform2d(nearestTag.pose.toPose2d(), pose);

            System.out.println("Tag: " + nearestTag.ID +" Transform: " + transform);
        }
    }

    private AprilTag getNearestReefAprilTag(Pose2d robotPose) {
        double minDistance = Double.MAX_VALUE;
        AprilTag nearestTag = null;

        for (AprilTag tag: Constants.apriltagLayout.getTags()) {
                if (Arrays.binarySearch(Constants.CoralPlacement.REEF_TAGS, tag.ID) >= 0) {
                        double distance = tag.pose.toPose2d().getTranslation().getDistance(robotPose.getTranslation());
                        if (Math.abs(distance) < minDistance) {
                                minDistance = Math.abs(distance);
                                nearestTag = tag;
                        }
                }
        }

        return nearestTag;
    }

}
