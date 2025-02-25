package frc.robot.subsystems;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.subsystems.AutoAiming;
import frc.robot.util.RobotMechanism;

/** Add your docs here. */
public class AutoAimingTest {
    @Test
    public void testZeros() throws Exception {
        AutoAiming autoAim = new AutoAiming(()->{return new Pose3d(0.0,0.0,100.0,new Rotation3d());});

        Rotation2d hAngle = autoAim.horizontalRotationToCoral();
        Rotation2d vAngle = autoAim.verticalRotationToCoral();
        Pose2d coralBranch = autoAim.chooseBranch(new Pose2d());
        System.out.println("horizontal angle " + hAngle);
        System.out.println("vertical angle " +vAngle);
        System.out.println(coralBranch);
    }
}
