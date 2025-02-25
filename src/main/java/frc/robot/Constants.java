package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

import java.io.IOException;
import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

public final class Constants {
  public static final String canivoreBusName = "rio";
  public static final AprilTagFieldLayout apriltagLayout;
  public static final Translation2d fieldSize;

  static {
    try {
      apriltagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025Reefscape.m_resourceFile);
      apriltagLayout.getFieldLength();
      apriltagLayout.getFieldWidth();
      apriltagLayout.getFieldLength();
      apriltagLayout.getFieldWidth();
      fieldSize = new Translation2d(apriltagLayout.getFieldLength(), apriltagLayout.getFieldWidth());
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  /**
   * Annotate CAN ID fields with this annotation so we can detect duplicates in a unit test.
   */
  @Retention(RetentionPolicy.RUNTIME)
  @Target(ElementType.FIELD)
  public @interface CanId {}

  public static final class Elevator {
    public static final int motorCanID = 41;

    public static final double P = 0.05;
    public static final double I = 0.00;
    public static final double D = 0.00;
    public static final double FF = 1.0/(565.0*12);

    public static final double METERS_PER_MOTOR_REVOLUTION = 2 * Units.inchesToMeters(1.0 / 8.0);
    public static final double ELEVATOR_MASS = Units.lbsToKilograms(20.0);
    public static final double GEAR_RATIO = 1.0;
    public static final double MIN_HEIGHT_METERS = 0.0;
    public static final double MAX_HEIGHT_METERS = Units.inchesToMeters(56.0);
    public static final double STARTING_HEIGHT_METERS = MIN_HEIGHT_METERS + (MIN_HEIGHT_METERS + MAX_HEIGHT_METERS) / 2.0;

    public static final double level1Height = 0;
    public static final double level2Height = 0;
    public static final double level3Height = 0;
    public static final double level4Height = 0;
    public static final double MAX_ACCELERATION = 24000.0;
    public static final double MAX_VELOCITY = 6000.0;
  };

  public static final class LEDs {
    public static final int stripPwm = 0;
    public static final int stripLength = 150;
  }

  public static final class Swerve {
    @CanId public static final int imuCanID = 3;
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
      @CanId public static final int driveMotorCanID = 7;
      @CanId public static final int rotationMotorCanID = 8;
      @CanId public static final int rotationEncoderCanID = 13;
      public static Translation2d moduleOffset = new Translation2d(Units.inchesToMeters(11.25),
          Units.inchesToMeters(12.25));
    }

    public static final class FrontRightModule {
      @CanId public static final int driveMotorCanID = 10;
      @CanId public static final int rotationMotorCanID = 11;
      @CanId public static final int rotationEncoderCanID = 22;
      public static Translation2d moduleOffset = new Translation2d(Units.inchesToMeters(11.25),
          -Units.inchesToMeters(12.25));
    }

    public static final class BackLeftModule {
      @CanId public static final int driveMotorCanID = 4;
      @CanId public static final int rotationMotorCanID = 5;
      @CanId public static final int rotationEncoderCanID = 14;
      public static Translation2d moduleOffset = new Translation2d(-Units.inchesToMeters(11.25),
          Units.inchesToMeters(12.25));
    }

    public static final class BackRightModule {
      @CanId public static final int driveMotorCanID = 6;
      @CanId public static final int rotationMotorCanID = 9;
      @CanId public static final int rotationEncoderCanID = 15;
      public static Translation2d moduleOffset = new Translation2d(-Units.inchesToMeters(11.25),
          -Units.inchesToMeters(12.25));
    }
  }

  public static final class CoralHandler {
    // TODO figure out all actual constants
    public static final double MeterPerMotorRevolution = 0.0;

    @CanId public static final int outtakeMotorID = 71;
    @CanId public static final int horizontalMotorID = 72;
    @CanId public static final int horizontalEncoderID = 73;
    @CanId public static final int verticalMotorID = 74;
    @CanId public static final int verticalEncoderID = 75;

    // !! `coralEndEffectorLength` is IN METERS
    public static final double coralEndEffectorLength = 0.25;
    public static final double coralEndEffectorMass = 0.5;

    public static final double outtakeWheelMass = Units.lbsToKilograms(0.5);
    public static final double outtakeWheelRadius = 0.02;
    
    public static final double coralIntakeSpeed = 0;
    public static final double coralOuttakeSpeed = 0;
    
    public static final double outtakeMotorGearing = 1.0;
    public static final double horizontalGearRatio = 100.0;
    public static final double verticalGearRatio = 100.0;

    public static final double outtakeJKgMetersSquared = (.5 * outtakeWheelMass * Math.pow(outtakeWheelRadius, 2));
    public static final double outtakeMotorMinVelocity = 0.0;
    // public static final int outtakeEncoderID = 0.0;
    
    public static final double horizontalMotorP = 0.03;
    public static final double horizontalMotorI = 0.0;
    public static final double horizontalMotorD = 0.0;
    public static final double horizontalMotorFeedForward = 1.0 / (565.0*12.0);
    public static final double horizontalMotorIZone = 0.0;
    public static final double horizontalMotorMaxAccleration = 25000.0; //RPM per Sec
    public static final double horizontalMotorMaxVelocity = 6500.0; //RPM
    public static final double horizontalMotorClosedLoopError = 1.0;

    public static final double verticalMotorP = 0.03;
    public static final double verticalMotorI = 0.0;
    public static final double verticalMotorD = 0.0;
    public static final double verticalMotorFeedForward = 1.0 / (565.0*12.0);
    public static final double verticalMotorIZone = 0.0;
    public static final double verticalMotorMaxAccleration = 25000.0; //RPM per Sec
    public static final double verticalMotorMaxVelocity = 6500.0; //RPM
    public static final double verticalMotorClosedLoopError = 1.0;
    
    public static final double horizontalJKgMetersSquared = 1.0/3.0 * coralEndEffectorMass * Math.pow(coralEndEffectorLength, 2.0);
    public static final double verticalJKgMetersSquared = 1.0/3.0 * coralEndEffectorMass * Math.pow(coralEndEffectorLength, 2.0);
    
    public static final Rotation2d horizontalMinAngle = Rotation2d.fromDegrees(-100);
    public static final Rotation2d horizontalMaxAngle = Rotation2d.fromDegrees(100);

    public static final Rotation2d verticalMinAngle = Rotation2d.fromDegrees(-90);
    public static final Rotation2d verticalMaxAngle = Rotation2d.fromDegrees(90);
    
    public static final Rotation2d horizontalStartingAngleInRadians = Rotation2d.fromDegrees(-90);
    public static final Rotation2d verticalStartingAngleInRadians = Rotation2d.fromDegrees(-100);
    
    public static final double horizontalMotorStdDev = 0.0;
    public static final double verticalMotorStdDev = 0.0;
    
    public static final double horizontalMotorMinVelocity = 0.0;
    public static final double verticalMotorMinVelocity = 0.0;

    public static final double verticalRotationDegreesPerRotation = 360 / verticalGearRatio;
    public static final double horizontalRotationDegreesPerRotation = 360 / horizontalGearRatio;
    
    //Need different name, for manual coral joystick control
    public static final double verticalAngleChangeDegreesPerSecond = verticalMotorMaxVelocity * verticalGearRatio / 60;
    
    public static final Rotation2d horizontalIntakeAngle = Rotation2d.fromDegrees(0);
    public static final Rotation2d horizontalLevel1AngleRight = Rotation2d.fromDegrees(0);
    public static final Rotation2d horizontalLevel1AngleLeft = Rotation2d.fromDegrees(0);
    public static final Rotation2d horizontalLevel2Angle = Rotation2d.fromDegrees(0);
    public static final Rotation2d horizontalLevel3Angle = Rotation2d.fromDegrees(0);
    public static final Rotation2d horizontalLevel4Angle = Rotation2d.fromDegrees(0);
    public static final Rotation2d vertialIntakeAngle = Rotation2d.fromDegrees(0);
    public static final Rotation2d vertialLevel1Angle = Rotation2d.fromDegrees(0);
    public static final Rotation2d verticallLevel2Angle = Rotation2d.fromDegrees(0);
    public static final Rotation2d verticallLevel3Angle = Rotation2d.fromDegrees(0);
    public static final Rotation2d verticallLevel4Angle = Rotation2d.fromDegrees(0);
    // ???? what do you do for offset for position (use gear ratio to figure out)
    // public static final double RotationDegreesPerRotation = 0;
  }

  public static final class AutoBalance {
    // public static final PIDConstants BALANCE_CONSTANTS = new PIDConstants(0.3,
    // 0.0, 0.1);
    public static final double maxVelAuto = 0.4;
    public static final double maxVelTele = 0.3;
  }

  public static final class Climber {
    public static final double firstStageGearRatio = 90 / 60;
    public static final double secondStageGearRatio = 18 / 58;
    
    public static final double MOTOR_KI = 0; // TODO
    public static final double MOTOR_KP = 0.25;
    public static final double MOTOR_KD = 0;
    public static final double GEAR_RATIO = 0.01;
    public static final double ARM_ANGULAR_MOMENTUM = Units.lbsToKilograms(9.963);
    public static final double LENGTH_METERS = Units.inchesToMeters(16.785);
    public static final double MIN_ANGLE_RADS = 0;
    public static final double MAX_ANGLE_RADS = 3 * Math.PI / 4;
    @CanId public static final int MOTOR_CANID = 51;
    public static final int PCMID = 5;
    public static final int SOLONOIDID = 3;
    @CanId public static final int climberEncoderCanID = 12; //TODO
  }
 
public static final class AlgaeHandler {
  //Creating constants for LEFT Algae Handler :D
  //CANID's
  @CanId  static final int leftAlgaeMotorCANID = 21;
  public static final int leftAlgaeSolenoidID = 3;
  public static final int leftAlgaeHallEffectID = 23;
  public static final int leftAlgaeLimitID = 24;

    //Creating constants for RIGHT Algae Handler :D
    @CanId public static final int rightAlgaeMotorCANID = 25;
  public static final int rightAlgaeSolenoidID = 4;
  public static final int rightAlgaeHallEffectID = 27;
  public static final int rightAlgaeLimitID = 28;
  

  //all of these ID's are place holders and will need to be edited at a later date
  public static final int degreesPerRevolution = 360;
  //These values will need to be changed, just place holders
  public static final double algaeMotorP = 0.001;
  public static final double algaeMotorI = 0.00;
  public static final double algaeMotorD = 0.000;
  public static final double algaeFF = 1.0/(565.0*12);
  public static final double algaeIZone = 0.0;
  public static final double algaeMotorMaxVelocity = 6000.0;
  public static final double algaeMotorMaxAcceleration = 0.0;
  public static final double algaeMotorAllowedError = 1;
  //Calculates moment of inertia for parameter in flywheel sim for bottom wheels 
  public static final double massOfBottomIntakeWheel = Units.lbsToKilograms(0.076);
  public static final double radiusOfBottomIntakeWheel = .025;
  public static final double momentOfInertiaOfTheBottomIntakeWheel = .5 * (massOfBottomIntakeWheel * (radiusOfBottomIntakeWheel*radiusOfBottomIntakeWheel)); 

  //Calculates moment of inertia for parameter in flywheel sim for top wheels
  public static final double massOfTopIntakeWheel = Units.lbsToKilograms(0.035);
  public static final double radiusOfTopIntakeWheel = 1;
  public static final double momentOfInertiaOfTheTopIntakeWheel = massOfTopIntakeWheel * (radiusOfTopIntakeWheel*radiusOfTopIntakeWheel);



    //all of these ID's are place holders and will need to be edited at a later date

    public static final double metersPerMotorRevolution = 0;
    public static final int amassOfAlgaeHandler = 6;
    public static final double algaeGearRatio = 1.0/9.0;
    




//Motor logistics

  





  
}
  
}
