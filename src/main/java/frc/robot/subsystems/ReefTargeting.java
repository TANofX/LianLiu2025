// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ReefTargeting extends SubsystemBase {
  private Supplier<Pose2d> robotPoseSupplier;
  private AprilTag targetTag;
  private BranchPosition targetPosition = BranchPosition.NONE;

  public enum BranchPosition {
    LEFT,
    RIGHT,
    NONE
  }

  /** Creates a new ReefTargeting. */
  public ReefTargeting(Supplier<Pose2d> poseSupplier) {
    robotPoseSupplier = poseSupplier;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setTargetAprilTag(BranchPosition targetPosition) {
    targetTag = getNearestReefAprilTag();
    this.targetPosition = targetPosition;
  }

  public void clearTargetAprilTag() {
    targetTag = null;
    targetPosition = BranchPosition.NONE;
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

    return targetTag.pose.toPose2d().plus(transform);
  }

  /*
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
  */

  public Optional<Pose2d> getRobotTargetPose2d() {
    if(targetPosition == BranchPosition.LEFT){
      return Optional.ofNullable(adjustTargetAprilTag(Constants.CoralPlacement.LEFT_CORAL_ROBOT_OFFSET_FROM_APRILTAG));
    }else if(targetPosition == BranchPosition.RIGHT){
      return Optional.ofNullable(adjustTargetAprilTag(Constants.CoralPlacement.RIGHT_CORAL_ROBOT_OFFSET_FROM_APRILTAG));
    }else{
      return Optional.empty();
    }
  }

  public Optional<Pose2d> getBranchTargetPose2d() {
    if(targetPosition == BranchPosition.LEFT){
      return Optional.ofNullable(adjustTargetAprilTag(Constants.CoralPlacement.LEFT_CORAL_APRILTAG_OFFSET));
    }else if(targetPosition == BranchPosition.RIGHT){
      return Optional.ofNullable(adjustTargetAprilTag(Constants.CoralPlacement.RIGHT_CORAL_APRILTAG_OFFSET));
    }else{
      return Optional.empty();
    }
  }
}
