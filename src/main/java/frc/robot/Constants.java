package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

import java.io.IOException;

public final class Constants {
  public static final String canivoreBusName = "rio";
  public static final AprilTagFieldLayout apriltagLayout;

  static {
    try {
      apriltagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025Reefscape.m_resourceFile);
      apriltagLayout.getFieldLength();
      apriltagLayout.getFieldWidth();
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  public static final Translation2d fieldSize = new Translation2d(16.54, 8.02);
  public static final double noteTransferMetersPerSecond = 0.5;

  public static final class LEDs {
    public static final int stripPwm = 0;
    public static final int stripLength = 150;
  }

  public static final class Swerve {
    public static final int imuCanID = 3;
    public static final double maxVelTele = 4.7;
    public static final double maxAccelTele = 6.0;
    public static final double maxAngularVelTele = Units.degreesToRadians(180);
    public static final double maxAngularAccelTele = Units.degreesToRadians(540);
    public static final double teleAngleHoldFactor = 3.0;

    public static final double SPEAKER_CONTROLLER_kP = 1.0;
    public static final double SPEAKER_CONTROLLER_kI = 0.0;
    public static final double SPEAKER_CONTROLLER_kD = 0.01;

    public static final class Odometry {
      public static final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.05);
      public static final Matrix<N3, N1> visionStdDevs = VecBuilder.fill(0.9, 0.9, 0.9);
    }

    public static final class PathFollowing {
      // public static final PIDConstants TRANSLATION_CONSTANTS = new
      // PIDConstants(5.0, 0.0, 0.0);
      // public static final PIDConstants ROTATION_CONSTANTS = new PIDConstants(2.0,
      // 0.0, 0.0);
    }

    public static final class FrontLeftModule {
      public static final int driveMotorCanID = 7;
      public static final int rotationMotorCanID = 8;
      public static final int rotationEncoderCanID = 13;
      public static Translation2d moduleOffset = new Translation2d(Units.inchesToMeters(11.25),
          Units.inchesToMeters(12.25));
    }

    public static final class FrontRightModule {
      public static final int driveMotorCanID = 10;
      public static final int rotationMotorCanID = 11;
      public static final int rotationEncoderCanID = 12;
      public static Translation2d moduleOffset = new Translation2d(Units.inchesToMeters(11.25),
          -Units.inchesToMeters(12.25));
    }

    public static final class BackLeftModule {
      public static final int driveMotorCanID = 4;
      public static final int rotationMotorCanID = 5;
      public static final int rotationEncoderCanID = 14;
      public static Translation2d moduleOffset = new Translation2d(-Units.inchesToMeters(11.25),
          Units.inchesToMeters(12.25));
    }

    public static final class BackRightModule {
      public static final int driveMotorCanID = 6;
      public static final int rotationMotorCanID = 9;
      public static final int rotationEncoderCanID = 15;
      public static Translation2d moduleOffset = new Translation2d(-Units.inchesToMeters(11.25),
          -Units.inchesToMeters(12.25));
    }
  }

  public static final class AutoBalance {
    // public static final PIDConstants BALANCE_CONSTANTS = new PIDConstants(0.3,
    // 0.0, 0.1);
    public static final double maxVelAuto = 0.4;
    public static final double maxVelTele = 0.3;
  }

 

  
}
