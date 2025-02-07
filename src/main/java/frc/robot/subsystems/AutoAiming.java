// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose3d;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance; //may not need both?

public class AutoAiming extends SubsystemBase {
  /** Creates a new AutoAiming. */
  public AutoAiming() {
  }


//Method to return the position of the branch closest to it.
  public static Pose2d chooseBranch (Pose2d position){
    Alliance alliance = DriverStation.getAlliance().orElseGet(() -> Alliance.Blue);

    List<Pose2d> coordinates;
    if (alliance == Alliance.Red) {
        coordinates = Constants.CoralPlacement.cordinatesCoralRed;
    } else {
        coordinates = Constants.CoralPlacement.cordinatesCoralBlue;
    }
    return position.nearest(coordinates);
  }

  
  public static Rotation2d rotationToCoral (Pose2d position){
    Transform2d changeNeeded = new Transform2d(position, AutoAiming.chooseBranch(Swerve.getPose()));
        return changeNeeded;
    
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
