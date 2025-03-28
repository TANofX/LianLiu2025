// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ReefTargeting extends SubsystemBase {
  private Supplier<Pose2d> robotPoseSupplier;
  private AprilTag targetTag;
  private final StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault().getStructTopic(getName() + "targetPose", Pose2d.struct).publish();

  /** Creates a new ReefTargeting. */
  public ReefTargeting(Supplier<Pose2d> poseSupplier) {
    robotPoseSupplier = poseSupplier;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setTargetAprilTag() {
    targetTag = getNearestReefAprilTag();
    SmartDashboard.putNumber(getName() + "/aprilTag", targetTag.ID);
    
  }

  public void clearTargetAprilTag() {
    targetTag = null;
  }

  private AprilTag getNearestReefAprilTag() {
    double minDistance = Double.MAX_VALUE;
    AprilTag nearestTag = null;
    Pose2d robotPose = robotPoseSupplier.get();

    for (AprilTag tag : Constants.apriltagLayout.getTags()) {
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

  private Pose2d adjustTargetAprilTag(Transform2d transform) {
    if (targetTag == null) {
      return null;
    }

    Pose2d pose = targetTag.pose.toPose2d().plus(transform);
    publisher.set(pose);
    return pose;
  }

  public Pose2d getLeftCoralTargetPose() {
    return adjustTargetAprilTag(Constants.CoralPlacement.LEFT_CORAL_ROBOT_OFFSET_FROM_APRILTAG);
  }

  public Pose2d getRightCoralTargetPose() {
    return adjustTargetAprilTag(Constants.CoralPlacement.RIGHT_CORAL_ROBOT_OFFSET_FROM_APRILTAG);
  }

  public Pose2d getLeftCoralBranchPose() {
    return adjustTargetAprilTag(Constants.CoralPlacement.LEFT_CORAL_APRILTAG_OFFSET);
  }

  public Pose2d getRightCoralBranchPose() {
    return adjustTargetAprilTag(Constants.CoralPlacement.RIGHT_CORAL_APRILTAG_OFFSET);
  }
}
