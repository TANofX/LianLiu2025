// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance; //may not need both?
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoAiming extends SubsystemBase {
  /** Creates a new AutoAiming. */
  private final Supplier<Pose3d> coralHandlerSupplier;
  private final StructPublisher<Rotation2d> publisher = NetworkTableInstance.getDefault().getStructTopic("AutoAiming/Horizontal", Rotation2d.struct).publish(); 
  private final StructPublisher<Translation2d> publisher2 = NetworkTableInstance.getDefault().getStructTopic("AutoAiming/Horizontal", Translation2d.struct).publish();
  private final StructPublisher<Translation2d> publisher3 = NetworkTableInstance.getDefault().getStructTopic("AutoAiming/Horizontal", Translation2d.struct).publish();
  
  public AutoAiming(Supplier<Pose3d> coralHandlerPoseSupplier) {
    this.coralHandlerSupplier = coralHandlerPoseSupplier;
  }


//Method to return the position of the branchCon closest to it.
  public Translation2d chooseBranch (Translation2d robotPose){
    double closest = 10000000000000.0;
    int ind = -1;
    for(int i = 0; i<Constants.CoralPlacement.coordinatesCoral.size()-0; i++){
      
      if(Constants.CoralPlacement.coordinatesCoral.get(i).getDistance(robotPose) < closest){
        closest = Constants.CoralPlacement.coordinatesCoral.get(i).getDistance(robotPose);
        ind = i;
      }
    }
    return Constants.CoralPlacement.coordinatesCoral.get(ind);
  }
  //Need to create two different methods to determine the verticle rotation and horizontal rotation
  //put in two pose2ds (from translation2ds), and turn these into transform 2ds, 
  //turn this into translation 2ds, and turn this into rotation 2ds
  
  public Rotation2d horizontalRotationToCoral (){
    Translation2d robotPose = coralHandlerSupplier.get().toPose2d().getTranslation();
    Translation2d changeNeeded = chooseBranch(robotPose).minus(robotPose);
    Rotation2d robotRotation = coralHandlerSupplier.get().getRotation().toRotation2d();

//IN DEGREES NOT RADIANS!!!!
    Rotation2d initialAngle = new Rotation2d(-(robotRotation.getDegrees() - changeNeeded.getAngle().getDegrees()));
    Rotation2d finalAngle;
    

    
    if ((initialAngle.getDegrees() < 80) && (initialAngle.getDegrees() > (-80))){
      finalAngle = changeNeeded.getAngle();
    }else if(initialAngle.getDegrees() > 280){
      finalAngle = new Rotation2d(initialAngle.getDegrees()-360);
    }else if(initialAngle.getDegrees() < -280){
      finalAngle = new Rotation2d(initialAngle.getDegrees()+360);
    }else{
      finalAngle = RobotContainer.coralHandler.getHorizontalAngle();
    }
  
    finalAngle = initialAngle;

    return finalAngle;
  }



//NOT DONEEE
public Rotation2d verticalRotationToCoral (){
  double horizontalChangeNeeded = coralHandlerSupplier.get().getTranslation().toTranslation2d().getDistance(chooseBranch(coralHandlerSupplier.get().toPose2d().getTranslation()));
  
  double closest = 100000.0;
  for(int i = 0; i<stants.CoralPlacement.heightsCoral.size(); i++){
    if(Constants.CoralPlacement.heightsCoral.get(i) < closest){
      closest = Constants.CoralPlacement.heightsCoral.get(i);
    }
  }
  
  Translation2d sideViewCoral = new Translation2d(horizontalChangeNeeded, closest - coralHandlerSupplier.get().getZ());
  return sideViewCoral.getAngle();
}


//need to pass in robot base
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Rotation2d temp = horizontalRotationToCoral();
    publisher.set(temp);
    
  }
}

//go in straight line while raising elevator to test if we can raise elevator while driving


