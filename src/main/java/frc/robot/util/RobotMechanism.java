// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.Elevator;
import frc.robot.RobotContainer;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
//import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
//import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
//import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructArrayPublisher;

/** Add your docs here. */
public class RobotMechanism {
    Mechanism2d m_mechanism;
    MechanismRoot2d m_root;
    MechanismLigament2d m_elevatorExtension;
    StructArrayPublisher<Pose3d> m_poses = NetworkTableInstance.getDefault().getStructArrayTopic("Robot/Component/Poses", Pose3d.struct).publish();
    Pose3d[] poses = new Pose3d[9];

    private static final Pose3d k_RobotBaseOffset = new Pose3d(0, 0, 0.0661163, new Rotation3d());
    private static final Transform3d k_RobotToElevatorStage2 = new Transform3d(0.1909, 0.0, 0.06075, new Rotation3d());
    private static final Transform3d k_ElevatorStage2ToStage3 = new Transform3d(0.0, 0.0, 0.0346, new Rotation3d());
    private static final Transform3d k_ElevatorStage3ToCoralHandlerStage1 = new Transform3d(0.1136, 0.0, 0.6501, new Rotation3d());
    private static final Transform3d k_CoralHandlerStage1ToStage2 = new Transform3d(0.0956, 0.0, 0.0089, new Rotation3d());
    private static final Transform3d k_LeftAlgaeHandler = new Transform3d(0.0005398, 0.29562, 0.13011, new Rotation3d(0.0, 0.0, Math.PI / 2.0));
    private static final Transform3d k_RightAlgaeHandler = new Transform3d(0.0005398, -0.29562, 0.13011, new Rotation3d(0.0, 0.0, -Math.PI / 2.0));
    private static final Transform3d k_ClimberClamShell = new Transform3d(-0.30481, 0.01196, 0.25714, new Rotation3d(0.0, 0.0, Math.PI));
    private static final Transform3d k_ClimberClam2LeftArm = new Transform3d(0.15222, 0.1125, 0.01433, new Rotation3d());
    private static final Transform3d k_ClimberClam2RightArm = new Transform3d(0.15222, -0.1125, 0.01433, new Rotation3d());

    // TODO Investigate Mechanism2d
    // Does this gets drawn correctly in SmartDashboard? AdvantageScope did not seem to draw the mechanism in a manner
    // to understand the robot's configuration.
    public RobotMechanism() {
        m_mechanism = new Mechanism2d(Units.inchesToMeters(15.0), Units.inchesToMeters(5) + Elevator.MAX_HEIGHT_METERS);
        m_root = m_mechanism.getRoot("Elevator", Units.inchesToMeters(15.0 - 7.5), Units.inchesToMeters(4.0));
        MechanismLigament2d m_elevatorBase = new MechanismLigament2d("Elavator Extension", Units.inchesToMeters(5), 90.0, 10, new Color8Bit(200, 200, 200));
        m_elevatorExtension = new MechanismLigament2d("Elavator Base", Units.inchesToMeters(4), 90.0, 5, new Color8Bit(128, 128, 128));
        m_root.append(m_elevatorExtension);
        m_root.append(m_elevatorBase);

        for (int i = 0; i < poses.length; i++) {
            poses[i] = new Pose3d();
        }
    }

    public void update() {
        double elevation = RobotContainer.elevator.getElevation();
        m_elevatorExtension.setLength(elevation);

        Rotation2d coralHandlerAngle = RobotContainer.coralHandler.getHorizontalAngle();
        Rotation2d coralHandlerVerticalAngle = RobotContainer.coralHandler.getVerticalAngle();
        Rotation2d clamShellAngle = RobotContainer.climber.getCurrentAngle();

        double leftAlgaeHandlerAngle = RobotContainer.leftAlgaeHandler.isAlgaeIntakeUp() ? 0.0 : Math.PI / 180.0 * 55.0;
        double rightAlgaeHandlerAngle = RobotContainer.rightAlgaeHandler.isAlgaeIntakeUp() ? 0.0 : Math.PI / 180.0 * 55.0;

        double climberArmAngle = RobotContainer.climber.isClawOpen() ? Math.PI / 4.0 : 0.0;

        SmartDashboard.putData("Robot Mechanism", m_mechanism);
        
        Transform3d elevatorTransform = new Transform3d(0.0, 0.0, elevation / 2.0, new Rotation3d());
        Transform3d coralHandlerStage1 = new Transform3d(0,0,0,new Rotation3d(0.0, 0.0, coralHandlerAngle.getRadians()));
        Transform3d coralHandlerStage2 = new Transform3d(0,0,0, new Rotation3d(0, coralHandlerVerticalAngle.getRadians(), 0));
        Transform3d leftAlgaeHandler = new Transform3d(0,0,0, new Rotation3d(0, leftAlgaeHandlerAngle, 0));
        Transform3d rightAlgaeHandler = new Transform3d(0,0,0, new Rotation3d(0, rightAlgaeHandlerAngle, 0));
        Transform3d climberClamShell = new Transform3d(0,0,0, new Rotation3d(0, clamShellAngle.getRadians(), 0));
        Transform3d climberClam2LeftArm = new Transform3d(0,0,0, new Rotation3d(0, 0.0, climberArmAngle));
        Transform3d climberClam2RightArm = new Transform3d(0,0,0, new Rotation3d(0, 0.0, -climberArmAngle));

        poses[0] = k_RobotBaseOffset.plus(k_RobotToElevatorStage2).plus(elevatorTransform);
        poses[1] = poses[0].plus(k_ElevatorStage2ToStage3).plus(elevatorTransform);
        poses[2] = poses[1].plus(k_ElevatorStage3ToCoralHandlerStage1).plus(coralHandlerStage1);
        poses[3] = poses[2].plus(k_CoralHandlerStage1ToStage2).plus(coralHandlerStage2);
        poses[4] = k_RobotBaseOffset.plus(k_LeftAlgaeHandler).plus(leftAlgaeHandler);
        poses[5] = k_RobotBaseOffset.plus(k_RightAlgaeHandler).plus(rightAlgaeHandler);
        poses[6] = k_RobotBaseOffset.plus(k_ClimberClamShell).plus(climberClamShell);
        poses[7] = poses[6].plus(k_ClimberClam2LeftArm).plus(climberClam2LeftArm);
        poses[8] = poses[6].plus(k_ClimberClam2RightArm).plus(climberClam2RightArm);
        m_poses.set(poses);
    }

    //method that finds the position 
    public Pose3d getFieldPositionOfCoralHandler(){
        //turns Rotation2d into Rotation3d
        Rotation3d rotation = new Rotation3d(frc.robot.RobotContainer.swerve.getPose().getRotation());
        Translation3d baseToCoralHandler = poses[3].getTranslation();
        

        //this should now include any rotation of the drive base. This works because the pose3d constructor that uses both translation and rotation3d should rotate the axis so that it matches the rotation, and then construct the translation onto that new axis. 
        return new Pose3d(baseToCoralHandler, rotation);
    }
}
