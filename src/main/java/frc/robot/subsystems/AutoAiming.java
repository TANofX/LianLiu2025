// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.Constants;

public class AutoAiming extends SubsystemBase {
  /** Creates a new AutoAiming. */
  private final Supplier<Pose3d> coralHandlerSupplier;
  private final StructPublisher<Translation2d> publisher = NetworkTableInstance.getDefault().getStructTopic("AutoAiming/Horizontal", Translation2d.struct).publish();
  
  public AutoAiming(Supplier<Pose3d> coralHandlerPoseSupplier) {
    this.coralHandlerSupplier = coralHandlerPoseSupplier;
  }


//Method to return the position of the branch closest to it.
  public Pose2d chooseBranch (Pose2d robotPose){
    return robotPose.nearest( Constants.CoralPlacement.coordinatesCoral);
  }
  //Need to create two different methods to determine the verticle rotation and horizontal rotation
  //put in two pose2ds (from translation2ds), and turn these into transform 2ds, 
  //turn this into translation 2ds, and turn this into rotation 2ds
  
  public Translation2d horizontalRotationToCoral (){
    Translation2d changeNeeded = coralHandlerSupplier.get().getTranslation().toTranslation2d().minus(chooseBranch(coralHandlerSupplier.get().toPose2d()).getTranslation());
    // System.out.println("Auto Aiming: coral handler pose: " + coralHandlerSupplier.get() + "      ");
    // System.out.println("Auto Aiming: coral pose: " + chooseBranch(coralHandlerSupplier.get().toPose2d()));
    return changeNeeded;
  }
//NOT DONEEE
public Rotation2d verticalRotationToCoral (){
  double horizontalChangeNeeded = coralHandlerSupplier.get().getTranslation().toTranslation2d().getDistance(chooseBranch(coralHandlerSupplier.get().toPose2d()).getTranslation());
  
  double closest = 100000.0;
  for(int i = 0; i<Constants.CoralPlacement.heightsCoral.size(); i++){
    if(Constants.CoralPlacement.heightsCoral.get(i) < closest){
      closest = Constants.CoralPlacement.heightsCoral.get(i);
    }
  }
  
  Translation2d sideViewCoral = new Translation2d(horizontalChangeNeeded, closest - coralHandlerSupplier.get().getZ());
  // System.out.println(sideViewCoral);
  return sideViewCoral.getAngle();
}


//need to pass in robot base
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Translation2d temp = horizontalRotationToCoral();
    publisher.set(temp);
  }
}

//go in straight line while raising elevator to test if we can raise elevator while driving