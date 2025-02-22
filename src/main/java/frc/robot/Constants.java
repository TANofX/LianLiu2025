package frc.robot;

import java.io.IOException;
import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;
import java.util.ArrayList;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static final String CARNIVORE_BUS_NAME = "rio";
  public static final AprilTagFieldLayout apriltagLayout;
  public static final Translation2d fieldSize;

  static {
    try {
      apriltagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025Reefscape.m_resourceFile);
      fieldSize = new Translation2d(apriltagLayout.getFieldLength(), apriltagLayout.getFieldWidth());
      apriltagLayout.getFieldLength();
      apriltagLayout.getFieldWidth();
      apriltagLayout.getFieldLength();
      apriltagLayout.getFieldWidth();

    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  /**
   * Annotate CAN ID fields with this annotation so we can detect duplicates in a
   * unit test.
   */
  @Retention(RetentionPolicy.RUNTIME)
  @Target(ElementType.FIELD)
  public @interface CanId {
    /**
     * The type of device that this CAN ID is for.
     *
     * You can use the same CAN ID for two different devices of different types
     * (e.g.: a Spark MAX motor and a Spark FLEX motor, or a Spark MAX motor and an
     * encoder).
     * (This is because the real CAN ID is much larger, but WPILib gives us 6 bytes
     * for ID and uses the device ID
     * for the other bytes.)
     * We could be more specific than these types, but for now we expect to want to
     * use the same CAN ID for a motor
     * and a corresponding encoder, but not for two motors. This could change.
     */
    Type value();

    /**
     * The device types.
     */
    enum Type {
      MOTOR,
      ENCODER,
      PIGEON,
      PCM_CONTROLLER,
    }
  }

  public static final class Elevator {
    
    @CanId(CanId.Type.MOTOR)
    public static final int MOTOR_ID = 30;

    public static final double P = 0.05;
    public static final double I = 0.00;
    public static final double D = 0.00;
    public static final double FF = 1.0 / (565.0 * 12);

    public static final double METERS_PER_MOTOR_REVOLUTION = 2 * Units.inchesToMeters(1.0 / 8.0);
    public static final double ELEVATOR_MASS = Units.lbsToKilograms(20.0);
    public static final double GEAR_RATIO = 1.0;
    public static final double MIN_HEIGHT_METERS = 0.0;
    public static final double MAX_HEIGHT_METERS = Units.inchesToMeters(56.0);
    public static final double STARTING_HEIGHT_METERS = MIN_HEIGHT_METERS
        + (MIN_HEIGHT_METERS + MAX_HEIGHT_METERS) / 2.0;

    public static final double LEVEL1_HEIGHT = 1.25;
    public static final double LEVEL2_HEIGHT = 0; // Set correct height
    public static final double LEVEL3_HEIGHT = 0;
    public static final double LEVEL4_HEIGHT = 0;
    public static final double MAX_ACCELERATION = 24000.0;
    public static final double MAX_VELOCITY = 6000.0;
  };

  public static final class LEDs {
    public static final int PWM_PIN = 0;
    public static final int LENGTH = 150;
  }

  public static final class Swerve {
    @CanId(CanId.Type.PIGEON)
    public static final int IMU_ID = 5;
    public static final double TELEOP_MAX_VELOCITY = 4.6;
    public static final double TELEOP_MAX_ACCELERATION = 6.0; // todo
    public static final double TELEOP_MAX_ANGULAR_VELOCITY = Units.degreesToRadians(180);
    public static final double TELEOP_MAX_ANGULAR_ACCELERATION = Units.degreesToRadians(540);
    public static final double TELEOP_ANGLE_HOLD_FACTOR = 3.0;

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
      @CanId(CanId.Type.MOTOR)
      public static final int DRIVE_MOTOR_ID = 14;
      @CanId(CanId.Type.MOTOR)
      public static final int ROTATION_MOTOR_ID = 10;
      @CanId(CanId.Type.ENCODER)
      public static final int ROTATION_ENCODER_ID = 10;
      public static Translation2d moduleOffset = new Translation2d(Units.inchesToMeters(10.125),
          Units.inchesToMeters(12.375));
    }

    public static final class FrontRightModule {
      @CanId(CanId.Type.MOTOR)
      public static final int DRIVE_MOTOR_ID = 17;
      @CanId(CanId.Type.MOTOR)
      public static final int ROTATION_MOTOR_ID = 13;
      @CanId(CanId.Type.ENCODER)
      public static final int ROTATION_ENCODER_ID = 13;
      public static Translation2d moduleOffset = new Translation2d(Units.inchesToMeters(10.125),
          -Units.inchesToMeters(12.375));
    }

    public static final class BackLeftModule {
      @CanId(CanId.Type.MOTOR)
      public static final int DRIVE_MOTOR_ID = 15;
      @CanId(CanId.Type.MOTOR)
      public static final int ROTATION_MOTOR_ID = 11;
      @CanId(CanId.Type.ENCODER)
      public static final int ROTATION_ENCODER_ID = 11;
      public static Translation2d moduleOffset = new Translation2d(-Units.inchesToMeters(10.125),
          Units.inchesToMeters(12.375));
    }

    public static final class BackRightModule {
      @CanId(CanId.Type.MOTOR)
      public static final int DRIVE_MOTOR_ID = 16;
      @CanId(CanId.Type.MOTOR)
      public static final int ROTATION_MOTOR_ID = 12;
      @CanId(CanId.Type.ENCODER)
      public static final int ROTATION_ENCODER_ID = 12;
      public static Translation2d moduleOffset = new Translation2d(-Units.inchesToMeters(10.125),
          -Units.inchesToMeters(12.375));
    }
  }

  // Class to access the coordinates of the coral on the field.
  public static final class CoralPlacement {
    public final static ArrayList<Pose2d> coordinatesCoralRed = new ArrayList<>();
    static {
      // ordered in line from A-L
      // rotation degree part of Pos2D is the direction the robot has to face to be
      // flush against the reef for that branch
      coordinatesCoralRed.add(new Pose2d(544.87, 152.03, Rotation2d.fromDegrees(180)));
      coordinatesCoralRed.add(new Pose2d(544.87, 164.97, Rotation2d.fromDegrees(180)));
      coordinatesCoralRed.add(new Pose2d(535.08, 181.89, Rotation2d.fromDegrees(240)));
      coordinatesCoralRed.add(new Pose2d(523.90, 188.32, Rotation2d.fromDegrees(240)));
      coordinatesCoralRed.add(new Pose2d(504.39, 188.32, Rotation2d.fromDegrees(300)));
      coordinatesCoralRed.add(new Pose2d(493.16, 181.89, Rotation2d.fromDegrees(300)));
      coordinatesCoralRed.add(new Pose2d(483.44, 164.97, Rotation2d.fromDegrees(0.0)));
      coordinatesCoralRed.add(new Pose2d(483.44, 152.03, Rotation2d.fromDegrees(0.0)));
      coordinatesCoralRed.add(new Pose2d(493.16, 135.15, Rotation2d.fromDegrees(60)));
      coordinatesCoralRed.add(new Pose2d(504.39, 128.65, Rotation2d.fromDegrees(60)));
      coordinatesCoralRed.add(new Pose2d(523.90, 128.65, Rotation2d.fromDegrees(120)));
      coordinatesCoralRed.add(new Pose2d(535.08, 135.15, Rotation2d.fromDegrees(120)));
    }

    public final static ArrayList<Pose2d> coordinatesCoralBlue = new ArrayList<>();
    static {
      // ordered in line from A-L, even though this is "opposite" of blue
      coordinatesCoralBlue.add(new Pose2d(146.052, 164.97, Rotation2d.fromDegrees(0.0)));
      coordinatesCoralBlue.add(new Pose2d(146.052, 152.03, Rotation2d.fromDegrees(0.0)));
      coordinatesCoralBlue.add(new Pose2d(155.43, 135.15, Rotation2d.fromDegrees(60)));
      coordinatesCoralBlue.add(new Pose2d(166.65, 128.65, Rotation2d.fromDegrees(60)));
      coordinatesCoralBlue.add(new Pose2d(136.51, 128.65, Rotation2d.fromDegrees(120)));
      coordinatesCoralBlue.add(new Pose2d(197.69, 135.15, Rotation2d.fromDegrees(120)));
      coordinatesCoralBlue.add(new Pose2d(207.48, 152.03, Rotation2d.fromDegrees(180)));
      coordinatesCoralBlue.add(new Pose2d(207.48, 164.97, Rotation2d.fromDegrees(180)));
      coordinatesCoralBlue.add(new Pose2d(197.69, 181.89, Rotation2d.fromDegrees(240)));
      coordinatesCoralBlue.add(new Pose2d(186.51, 188.32, Rotation2d.fromDegrees(240)));
      coordinatesCoralBlue.add(new Pose2d(166.65, 188.32, Rotation2d.fromDegrees(300)));
      coordinatesCoralBlue.add(new Pose2d(155.43, 181.89, Rotation2d.fromDegrees(300)));
    }
  }

  public static final class CoralHandler {
    // TODO figure out all actual constants
    public static final double METER_PER_MOTOR_REVOLUTION = 0.0;

    @CanId(CanId.Type.MOTOR)
    public static final int OUTTAKE_MOTOR_ID = 40;
    @CanId(CanId.Type.MOTOR)
    public static final int HORIZONTAL_MOTOR_ID = 41;
    @CanId(CanId.Type.ENCODER)
    public static final int HORIZONTAL_ENCODER_ID = 41;
    @CanId(CanId.Type.MOTOR)
    public static final int VERTICAL_MOTOR_ID = 42;
    @CanId(CanId.Type.ENCODER)
    public static final int VERTICAL_ENCODER_ID = 42;

    // !! `coralEndEffectorLength` is IN METERS
    public static final double CORAL_END_EFFECTOR_LENGTH = 0.25;
    public static final double CORAL_END_EFFECTOR_MASS = 0.5;

    public static final double OUTTAKE_WHEEL_MASS = Units.lbsToKilograms(0.5);
    public static final double OUTTAKE_WHEEL_RADIUS = 0.02;

    public static final double CORAL_INTAKE_SPEED = 0;
    public static final double CORAL_OUTTAKE_SPEED = 0;

    public static final double OUTTAKE_MOTOR_GEARING = 1.0;
    public static final double HORIZONTAL_GEAR_RATIO = 100.0;
    public static final double VERTICAL_GEAR_RATIO = 100.0;

    public static final double OUTTAKE_JKMETERS_SQUARED = (.5 * OUTTAKE_WHEEL_MASS * Math.pow(OUTTAKE_WHEEL_RADIUS, 2));
    public static final double OUTTAKE_MOTOR_MIN_VELOCITY = 0.0;

    public static final double HORIZONTAL_MOTOR_POS_P = 0.03;
    public static final double HORIZONTAL_MOTOR_POS_I = 0.0;
    public static final double HORIZONTAL_MOTOR_POS_D = 0.0;
    public static final double HORIZONTAL_MOTOR_MAX_POS_P = 0.0;
    public static final double HORIZONTAL_MOTOR_MAX_POS_I = 0.0;
    public static final double HORIZONTAL_MOTOR_MAX_POS_D = 0.0;
    public static final double HORIZONTAL_MOTOR_POS_FEED_FORWARD = 1.0 / (565.0 * 12.0);
    public static final double HORIZONTAL_MOTOR_MAX_POS_FEED_FORWARD = 1.0 / (565.0 * 12.0);
    public static final double HORIZONTAL_MOTOR_POS_I_ZONE = 0.0;
    public static final double HORIZONTAL_MOTOR_MAX_POS_I_ZONE = 0.0;
    public static final double HORIZONTAL_MOTOR_MAX_ACCELERATION = 25000.0; // RPM per Sec
    public static final double HORIZONTAL_MOTOR_MAX_VELOCITY = 3500.0; // RPM
    public static final double HORIZONTAL_MOTOR_CLOSED_LOPP_ERROR = 1.0;

    public static final double VERTICAL_MOTOR_POS_P = 0.03;
    public static final double VERTICAL_MOTOR_POS_I = 0.0;
    public static final double VERTICAL_MOTOR_POS_D = 0.0;
    public static final double VERTICAL_MOTOR_MAX_POS_P = 0.03;
    public static final double VERTICAL_MOTOR_MAX_POS_I = 0.0;
    public static final double VERTICAL_MOTOR_MAX_POS_D = 0.0;
    public static final double VERTICAL_MOTOR_POS_FEED_FORWARD = 1.0 / (565.0 * 12.0);
    public static final double VERTICAL_MOTOR_MAX_POS_FEED_FORWARD = 1.0 / (565.0 * 12.0);
    public static final double VERTICAL_MOTOR_POS_I_ZONE = 0.0;
    public static final double VERTICAL_MOTOR_MAX_POS_I_ZONE = 0.0;
    public static final double VERTICAL_MOTOR_MAX_ACCELERATION = 25000.0; // RPM per Sec
    public static final double VERTICAL_MOTOR_MAX_VELOCITY = 3500.0; // RPM
    public static final double VERTICAL_MOTOR_CLOSED_LOOP_ERROR = 1.0;

    public static final double HORIZONTAL_JKMETERS_SQUARED = 1.0 / 3.0 * CORAL_END_EFFECTOR_MASS
        * Math.pow(CORAL_END_EFFECTOR_LENGTH, 2.0);
    public static final double VERTICAL_JKMETERS_SQUARED = 1.0 / 3.0 * CORAL_END_EFFECTOR_MASS
        * Math.pow(CORAL_END_EFFECTOR_LENGTH, 2.0);

    public static final Rotation2d HORIZONTAL_MIN_ANGLE = Rotation2d.fromDegrees(-100);
    public static final Rotation2d HORIZONTAL_MAX_ANGLE = Rotation2d.fromDegrees(100);

    public static final Rotation2d VERTICAL_MIN_ANGLE = Rotation2d.fromDegrees(-90);
    public static final Rotation2d VERTICAL_MAX_ANGLE = Rotation2d.fromDegrees(90);

    public static final Rotation2d HORIZONTAL_STARTING_ANGLE_IN_RADIANS = Rotation2d.fromDegrees(-90);
    public static final Rotation2d VERTICAL_STARTING_ANGLE_IN_RADIANS = Rotation2d.fromDegrees(-100);

    public static final double HORIZONTAL_MOTOR_STD_DEV = 0.0;
    public static final double VERTICAL_MOTOR_STD_DEV = 0.0;

    public static final double HORIZONTAL_MOTOR_MIN_VELOCITY = 0.0;
    public static final double VERTICAL_MOTOR_MIN_VELOCITY = 0.0;

    public static final double VERTICAL_ROTATION_DEGREES_PER_ROTATION = 360 / VERTICAL_GEAR_RATIO;
    public static final double HORIZONTAL_ROTATION_DEGREES_PER_ROTATION = 360 / HORIZONTAL_GEAR_RATIO;

    // Need different name, for manual coral joystick control
    public static final double VERTICAL_ANGLE_CHANGE_DEGREES_PER_SECOND = (VERTICAL_MOTOR_MAX_VELOCITY
        * VERTICAL_GEAR_RATIO) / 60;
    public static final double HORIZONTAL_ANGLE_CHANGE_DEGREES_PER_SECOND = (HORIZONTAL_MOTOR_MAX_VELOCITY
        * HORIZONTAL_GEAR_RATIO) / 60;

    public static final Rotation2d HORIZONTAL_INTAKE_ANGLE = Rotation2d.fromDegrees(0);
    public static final Rotation2d HORIZONTAL_LEVEL_1_ANGLE_RIGHT = Rotation2d.fromDegrees(0);
    public static final Rotation2d HORIZONTAL_LEVEL_1_ANGLE_LEFT = Rotation2d.fromDegrees(0);
    public static final Rotation2d HORIZONTAL_LEVEL_2_ANGLE = Rotation2d.fromDegrees(0);
    public static final Rotation2d HORIZONTAL_LEVEL_3_ANGLE = Rotation2d.fromDegrees(0);
    public static final Rotation2d HORIZONTAL_LEVEL_4_ANGLE = Rotation2d.fromDegrees(0);
    public static final Rotation2d VERTICAL_INTAKE_ANGLE = Rotation2d.fromDegrees(0);
    public static final Rotation2d VERTICAL_LEVEL_1_ANGLE = Rotation2d.fromDegrees(0);
    public static final Rotation2d VERTICAL_LEVEL_2_ANGLE = Rotation2d.fromDegrees(0);
    public static final Rotation2d VERTICAL_LEVEL_3_ANGLE = Rotation2d.fromDegrees(0);
    public static final Rotation2d VERTICAL_LEVEL_4_ANGLE = Rotation2d.fromDegrees(0);

  }

  public static final class Climber {
    @CanId(CanId.Type.MOTOR)
    public static final int CLIMBER_MOTOR_ID = 51;
    @CanId(CanId.Type.ENCODER)
    public static final int ENCODER_ID = 51;
    @CanId(CanId.Type.PCM_CONTROLLER)
    public static final int PCM_ID = 5;
    public static final int FORWARD_SOLENOID_ID = 14;
    public static final int REVERSE_SOLENOID_ID = 7;
    public static final double MOTOR_KP = .05;
    public static final double MOTOR_KI = 0; // TODO Configure Climber PID
    public static final double MOTOR_KD = 0.00025;
    public static final double MOTOR_FF = 1.0 / 6000.0;
    public static final double MOTOR_MAX_VELOCITY = 1000.0;
    public static final double MOTOR_MAX_ACCEL = 500.0;
    public static final double GEAR_RATIO = .0045977011494;
    public static final double ARM_ANGULAR_MOMENTUM = Units.lbsToKilograms(9.963);
    public static final double LENGTH_METERS = Units.inchesToMeters(4.785);
    public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(-156.0);
    public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(0);
    public static final int FORWARDSOLENOID = 14;
    public static final int REVERSESOLENOID = 7;

    public static final double CLIMBER_DEGREES_PER_ROTATION = 360 / GEAR_RATIO;
    public static final int ENCODERID = 42;
  }

  public static final class AlgaeHandler {
    // Creating constants for LEFT Algae Handler :D
    @CanId(CanId.Type.MOTOR)
    static final int LEFT_ALGAE_MOTOR_ID = 20;
    public static final int LEFT_ALGAE_SOLENOID_ID = 15;
    public static final int LEFT_ALGAE_LIMIT_ID = 0;

    // Creating constants for RIGHT Algae Handler :D
    @CanId(CanId.Type.MOTOR)
    public static final int RIGHT_ALGAE_MOTOR_ID = 21;
    public static final int RIGHT_ALGAE_SOLENOID_ID = 13;
    public static final int RIGHT_ALGAE_LIMIT_ID = 1;

    // These values will need to be changed, just place holders
    public static final double ALGAE_MOTOR_P = 0.001;
    public static final double ALGAE_MOTOR_I = 0.00;
    public static final double ALGAE_MOTOR_D = 0.000;
    public static final double ALGAE_MOTOR_FF = 1.0 / (565.0 * 12);
    public static final double ALGAE_MOTOR_I_ZONE = 0.0;
    public static final double ALGAE_MOTOR_MAX_VELOCITY = 6000.0;
    public static final double ALGAE_MOTOR_MAX_ACCELERATION = 0.0;
    public static final double ALGAE_MOTOR_ALLOWED_CLOSED_LOOP_ERROR = 1;
    // Calculates moment of inertia for parameter in flywheel sim for bottom wheels
    public static final double MASS_OF_BOTTOM_INTAKE_WHEEL = Units.lbsToKilograms(0.076);
    public static final double RADIUS_OF_BOTTOM_INTAKE_WHEEL = .025;
    public static final double MOMENT_OF_INERTIA_OF_THE_BOTTOM_INTAKE_WHEEL = .5
        * (MASS_OF_BOTTOM_INTAKE_WHEEL * (RADIUS_OF_BOTTOM_INTAKE_WHEEL * RADIUS_OF_BOTTOM_INTAKE_WHEEL));

    // Calculates moment of inertia for parameter in flywheel sim for top wheels
    public static final double MASS_OF_TOP_OF_INTAKE_WHEEL = Units.lbsToKilograms(0.035);
    public static final double RADIUS_OF_TOP_INTAKE_WHEEL = 1;
    public static final double MOMENT_OF_INERTIA_OF_THE_TOP_INTAKE_WHEEL = MASS_OF_TOP_OF_INTAKE_WHEEL
        * (RADIUS_OF_TOP_INTAKE_WHEEL * RADIUS_OF_TOP_INTAKE_WHEEL);

    // all of these ID's are place holders and will need to be edited at a later
    // date
    public static final double METERS_PER_MOTOR_REVOLUTION = 0;
    public static final int AMASS_OF_ALGAE_HANDLER = 6;
    public static final double ALGAE_GEAR_RATIO = 1.0 / 9.0;
  }

}
