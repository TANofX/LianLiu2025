package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.lib.swerve.Mk4SwerveModuleProSparkFlex;
import frc.lib.util.Vector3;
import frc.robot.Constants;
import frc.robot.util.RobotPoseLookup;

public final class Swerve extends AdvancedSubsystem {
  protected final SwerveDrivePoseEstimator odometry;
  public final SwerveDriveKinematics kinematics;

  protected final Mk4SwerveModuleProSparkFlex[] modules;

  protected final Pigeon2 imu;
  protected final Pigeon2SimState imuSim;
  protected final StatusSignal<Angle> imuRollSignal;
  protected final StatusSignal<Angle> imuPitchSignal;
  protected final StatusSignal<Angle> imuYawSignal;
  // protected final StatusSignalValue<Double> imuAccelXSignal;
  // protected final StatusSignalValue<Double> imuAccelYSignal;
  protected final StatusSignal<LinearAcceleration> imuAccelZSignal;

  private final RobotPoseLookup<Pose2d> poseLookup;

  protected double teleopVelConstraint;
  protected double teleopAngularVelConstraint;

  protected final Field2d field2d = new Field2d();

  private final StructArrayPublisher<SwerveModuleState> ModuleStatesPublisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("/Swerve/States", SwerveModuleState.struct).publish();
  private final StructArrayPublisher<SwerveModuleState> targetModuleStatesPublisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("/Swerve/TargetStates", SwerveModuleState.struct).publish();
  private final StructPublisher<Rotation2d> gyroPublisher = NetworkTableInstance.getDefault()
      .getStructTopic("/Swerve/Gyro", Rotation2d.struct).publish();

  private RobotConfig config;

  public Swerve() {
    super("Swerve");
    poseLookup = new RobotPoseLookup<>();

    imu = new Pigeon2(Constants.Swerve.IMU_ID, Constants.CARNIVORE_BUS_NAME);
    Pigeon2Configuration imuConfig = new Pigeon2Configuration();
    imu.getConfigurator().apply(imuConfig);
    zeroIMU();

    imuSim = imu.getSimState();

    teleopVelConstraint = Constants.Swerve.TELEOP_MAX_VELOCITY;
    teleopAngularVelConstraint = Constants.Swerve.TELEOP_MAX_ANGULAR_VELOCITY;

    imuRollSignal = imu.getRoll();
    imuPitchSignal = imu.getPitch();
    imuYawSignal = imu.getYaw();
    // imuAccelXSignal = imu.getAccelerationX();
    // imuAccelYSignal = imu.getAccelerationY();
    imuAccelZSignal = imu.getAccelerationZ();

    modules = new Mk4SwerveModuleProSparkFlex[] {
        new Mk4SwerveModuleProSparkFlex(
            Mk4SwerveModuleProSparkFlex.ModuleCode.FL,
            Constants.Swerve.FrontLeftModule.DRIVE_MOTOR_ID,
            Constants.Swerve.FrontLeftModule.ROTATION_MOTOR_ID,
            Constants.Swerve.FrontLeftModule.ROTATION_ENCODER_ID,
            Constants.CARNIVORE_BUS_NAME), // FL
        new Mk4SwerveModuleProSparkFlex(
            Mk4SwerveModuleProSparkFlex.ModuleCode.FR,
            Constants.Swerve.FrontRightModule.DRIVE_MOTOR_ID,
            Constants.Swerve.FrontRightModule.ROTATION_MOTOR_ID,
            Constants.Swerve.FrontRightModule.ROTATION_ENCODER_ID,
            Constants.CARNIVORE_BUS_NAME), // FR
        new Mk4SwerveModuleProSparkFlex(
            Mk4SwerveModuleProSparkFlex.ModuleCode.BL,
            Constants.Swerve.BackLeftModule.DRIVE_MOTOR_ID,
            Constants.Swerve.BackLeftModule.ROTATION_MOTOR_ID,
            Constants.Swerve.BackLeftModule.ROTATION_ENCODER_ID,
            Constants.CARNIVORE_BUS_NAME), // BL
        new Mk4SwerveModuleProSparkFlex(
            Mk4SwerveModuleProSparkFlex.ModuleCode.BR,
            Constants.Swerve.BackRightModule.DRIVE_MOTOR_ID,
            Constants.Swerve.BackRightModule.ROTATION_MOTOR_ID,
            Constants.Swerve.BackRightModule.ROTATION_ENCODER_ID,
            Constants.CARNIVORE_BUS_NAME) // BR
    };

    kinematics = new SwerveDriveKinematics(
        Constants.Swerve.FrontLeftModule.moduleOffset,
        Constants.Swerve.FrontRightModule.moduleOffset,
        Constants.Swerve.BackLeftModule.moduleOffset,
        Constants.Swerve.BackRightModule.moduleOffset);

    odometry = new SwerveDrivePoseEstimator(
        kinematics,
        getYaw(),
        getPositions(),
        new Pose2d(),
        Constants.Swerve.Odometry.stateStdDevs,
        Constants.Swerve.Odometry.visionStdDevs);

    registerHardware("IMU", imu);

    // SmartDashboard.putData("Tune Drive Motor", DriveTuner);
    // SmartDashboard.putData("Tune Steer Motor", SteerTuner);
    SmartDashboard.putData("Field", field2d);
    SmartDashboard.putData("Trim Modules", zeroModulesCommand());

    try {
      config = RobotConfig.fromGUISettings();

      AutoBuilder.configure(
          this::getPose,
          this::resetOdometry,
          this::getCurrentSpeeds,
          this::driveRobotRelative,
          new PPHolonomicDriveController(Constants.Swerve.PathFollowing.TRANSLATION_CONSTANTS,
              Constants.Swerve.PathFollowing.ROTATION_CONSTANTS),
          config,
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red
            // alliance.  This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          modules);
    } catch (Exception e) {
      DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
    }
    SmartDashboard.putData("Check Swerve", systemCheckCommand());
  }

  public Command goToPoseCommand(Pose2d targetPose, Rotation2d approachAngle) {
    Rotation2d targetRotation = targetPose.getRotation();
    Pose2d targetPoseWithApproach = new Pose2d(targetPose.getTranslation(), approachAngle);
    Rotation2d departureAngle = targetPose.getTranslation().minus(this.getPose().getTranslation()).getAngle();
    Pose2d departurePose = new Pose2d(this.getPose().getTranslation(), departureAngle);

    List<Pose2d> poses = new ArrayList<>();
    poses.add(departurePose);
    poses.add(targetPoseWithApproach);
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(poses);

    PathPlannerPath path = new PathPlannerPath(waypoints, 
                                                new PathConstraints(4.0, 2.0, 2 * Math.PI, 4 * Math.PI),
                                                new IdealStartingState(0.0, this.getPose().getRotation()), 
                                                new GoalEndState(0.0, targetRotation));
    path.preventFlipping = true;

    return AutoBuilder.followPath(path);
 }
 

  @Override
  public void periodic() {

    double startTime = Timer.getFPGATimestamp();

    Pose2d currentPose = odometry.update(getYaw(), getPositions());
    double correctTimeMS = (Timer.getFPGATimestamp() - startTime) * 1000;
    SmartDashboard.putNumber("Swerve/OdomRuntime", correctTimeMS);
    field2d.setRobotPose(currentPose);
    poseLookup.addPose(currentPose);
    SwerveModuleState[] moduleStates = new SwerveModuleState[] {
        modules[0].getState(),
        modules[1].getState(),
        modules[2].getState(),
        modules[3].getState()
    };
    ModuleStatesPublisher.set(moduleStates);

    SwerveModuleState[] targetModuleStates = new SwerveModuleState[] {
        modules[0].getTargetState(),
        modules[1].getTargetState(),
        modules[2].getTargetState(),
        modules[3].getTargetState()
    };

    targetModuleStatesPublisher.set(targetModuleStates);

    gyroPublisher.set(getYaw());
    ChassisSpeeds Chassis = getCurrentSpeeds();
    SmartDashboard.putNumberArray("Swerve/RobotVelocity",
        new double[] { Chassis.vxMetersPerSecond, Chassis.vyMetersPerSecond, Chassis.omegaRadiansPerSecond });
    SmartDashboard.putNumberArray(
        "Swerve/Odometry",
        new double[] {
            currentPose.getX(), currentPose.getY(), currentPose.getRotation().getDegrees()
        });
    SmartDashboard.putNumber("Swerve/FLEncoderAngle", modules[0].getAbsoluteRotationDegrees());
    SmartDashboard.putNumber("Swerve/FLRelativeAngle", modules[0].getRelativeRotationDegrees());
    // SmartDashboard.putNumber("OdometryX", currentPose.getX());
    // SmartDashboard.putNumber("OdometryY", currentPose.getY());
    // SmartDashboard.putNumber("OdometryR",
    // currentPose.getRotation().getDegrees());

    SmartDashboard.putNumber("FL", modules[0].getDriveVelocityMetersPerSecond());
    SmartDashboard.putNumber("FR", modules[1].getDriveVelocityMetersPerSecond());
    SmartDashboard.putNumber("BL", modules[2].getDriveVelocityMetersPerSecond());
    SmartDashboard.putNumber("BR", modules[3].getDriveVelocityMetersPerSecond());

    SmartDashboard.putNumberArray(
        "Swerve/ModuleStates",
        new double[] {
            modules[0].getAbsoluteRotationDegrees(),
            modules[0].getDriveVelocityMetersPerSecond(),
            modules[1].getAbsoluteRotationDegrees(),
            modules[1].getDriveVelocityMetersPerSecond(),
            modules[2].getAbsoluteRotationDegrees(),
            modules[2].getDriveVelocityMetersPerSecond(),
            modules[3].getAbsoluteRotationDegrees(),
            modules[3].getDriveVelocityMetersPerSecond(),
        });

    SmartDashboard.putNumberArray(
        "Swerve/TargetModuleStates",
        new double[] {
            modules[0].getTargetState().angle.getDegrees(),
            modules[0].getTargetState().speedMetersPerSecond,
            modules[1].getTargetState().angle.getDegrees(),
            modules[1].getTargetState().speedMetersPerSecond,
            modules[2].getTargetState().angle.getDegrees(),
            modules[2].getTargetState().speedMetersPerSecond,
            modules[3].getTargetState().angle.getDegrees(),
            modules[3].getTargetState().speedMetersPerSecond,
        });

    // Rotation3d orientation = getOrientation();
    // SmartDashboard.putNumberArray(
    // "Swerve/Orientation",
    // new double[] {
    // Units.radiansToDegrees(orientation.getX()),
    // Units.radiansToDegrees(orientation.getY()),
    // Units.radiansToDegrees(orientation.getZ())
    // });

    // Vector3 accel = getAcceleration();
    // SmartDashboard.putNumberArray(
    // "Swerve/Acceleration", new double[] {accel.getX(), accel.getY(),
    // accel.getZ()});
    // SmartDashboard.putNumber("Swerve/Pitch", getPitch());

    // WE MAY WANT THIS HELLO
    // if (!DriverStation.isAutonomousEnabled()) {
    // correctOdom(RobotContainer.driver.getBackButton(),
    // RobotContainer.driver.getBackButton());
    // }

    StatusSignal.waitForAll(0, imuRollSignal, imuPitchSignal, imuYawSignal, imuAccelZSignal);
    // imuRollSignal.refresh();
    // imuPitchSignal.refresh();
    // imuYawSignal.refresh();
    // imuAccelXSignal.refresh();
    // imuAccelYSignal.refresh();
    // imuAccelZSignal.refresh();

    double runtimeMS = (Timer.getFPGATimestamp() - startTime) * 1000;
    SmartDashboard.putNumber("Swerve/PeriodicRuntime", runtimeMS);

  }

  @Override
  public void simulationPeriodic() {
    ChassisSpeeds currentSpeeds = kinematics.toChassisSpeeds(getStates());
    double currentYaw = getYaw().getDegrees();

    imuSim.setRawYaw(
        currentYaw + (Units.radiansToDegrees(currentSpeeds.omegaRadiansPerSecond) * 0.02));
  }

  public Vector3 getGravityVector() {
    return new Vector3(
        imu.getGravityVectorX().getValue(),
        imu.getGravityVectorY().getValue(),
        imu.getGravityVectorZ().getValue());
  }

  /**
   * Drive the robot with field relative chassis speeds
   *
   * @param fieldRelativeSpeeds Field relative chassis speeds
   */
  public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        fieldRelativeSpeeds.vxMetersPerSecond,
        fieldRelativeSpeeds.vyMetersPerSecond,
        fieldRelativeSpeeds.omegaRadiansPerSecond,
        getPose().getRotation());

    driveRobotRelative(speeds);
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(speeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, Mk4SwerveModuleProSparkFlex.DRIVE_MAX_VEL);

    setModuleStates(targetStates);
  }

  public void driveRobotRelativeWithFF(ChassisSpeeds speeds, DriveFeedforwards ff) {
    SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(speeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, Mk4SwerveModuleProSparkFlex.DRIVE_MAX_VEL);

    setModuleStates(targetStates);
  }

  /** Zeros the IMU yaw */
  public void zeroIMU() {
    imu.setYaw(0);
  }

  /**
   * Get the yaw from the IMU
   *
   * @return Rotation2d representing the yaw of the robot
   */
  public Rotation2d getYaw() {
    return Rotation2d.fromDegrees(imuYawSignal.getValueAsDouble());
  }

  public double getPitch() {
    return imuPitchSignal.getValueAsDouble();
  }

  public double getRoll() {
    return imuRollSignal.getValueAsDouble();
  }

  /**
   * Get the 3d orientation of the robot
   *
   * @return Rotation3d representing the robot orientation
   */
  public Rotation3d getOrientation() {
    return new Rotation3d(
        Units.degreesToRadians(getRoll()),
        Units.degreesToRadians(getPitch()),
        getYaw().getRadians());
  }

  /**
   * Get the current acceleration of the robot from the IMU
   *
   * @return Vector3 representing the current acceleration
   */
  public Vector3 getAcceleration() {
    return new Vector3(
        imu.getAccelerationX().getValueAsDouble(),
        imu.getAccelerationY().getValueAsDouble(),
        imu.getAccelerationZ().getValueAsDouble());
  }

  /**
   * Get the current state of all the swerve modules
   *
   * @return Array of current swerve module states
   */
  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /**
   * Get the current positions of all the swerve modules
   *
   * @return Array of current swerve module positions
   */
  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      positions[i] = modules[i].getPositions();
    }
    return positions;
  }

  /**
   * Reset the odometry
   *
   * @param pose The pose to reset the odometry to
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getYaw(), getPositions(), pose);
  }

  /**
   * Get the estimated robot pose from the odometry
   *
   * @return Pose2d representing estimated robot pose
   */
  public Pose2d getPose() {
    return odometry.getEstimatedPosition();
  }

  /**
   * Set the desired states of all the swerve modules
   *
   * @param states Array of desired states, in the same order as the modules array
   */
  public void setModuleStates(SwerveModuleState[] states) {
    for (int i = 0; i < modules.length; i++) {
      modules[i].setDesiredState(states[i]);
    }
  }

  /**
   * Set the teleop driving constraints
   *
   * @param maxVel        Max robot velocity in m/s
   * @param maxAngularVel Max robot angular velocity in rad/s
   */
  public void setTeleopConstraints(double maxVel, double maxAngularVel) {
    teleopVelConstraint = maxVel;
    teleopAngularVelConstraint = maxAngularVel;
  }

  /**
   * Get the teleop velocity constraint
   *
   * @return Max teleop velocity in m/s
   */
  public double getTeleopVelConstraint() {
    return teleopVelConstraint;
  }

  /**
   * Get the teleop angular velocity constraint
   *
   * @return Max teleop velocity in rad/s
   */
  public double getTeleopAngularVelConstraint() {
    return teleopAngularVelConstraint;
  }

  public void lockModules() {
    for (Mk4SwerveModuleProSparkFlex module : modules) {
      module.lockModule();
    }
  }

  /**
   * Command used to trim all the modules' absolute encoders
   *
   * @return Command to trim the modules, runs while disabled
   */
  public Command zeroModulesCommand() {
    return Commands.runOnce(
        () -> {
          for (Mk4SwerveModuleProSparkFlex module : modules) {
            module.updateRotationOffset();
          }
        })
        .ignoringDisable(true);
  }

  public ChassisSpeeds getCurrentSpeeds() {
    return kinematics.toChassisSpeeds(getStates());
  }
 public Command backUpCommand() {
  return Commands.race(
    Commands.run(()-> {
      this.driveRobotRelative(new ChassisSpeeds(0.0 , -1.0, 0.0));
    }),
    Commands.waitSeconds(.30)

  );
 }
  @SuppressWarnings("removal")
  @Override
  protected Command systemCheckCommand() {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              modules[0].getSystemCheckCommand().schedule();
              modules[1].getSystemCheckCommand().schedule();
              modules[2].getSystemCheckCommand().schedule();
              modules[3].getSystemCheckCommand().schedule();
            },
            this),
        // Hack to run module system checks since modules[] does not exist when this
        // method is
        // called
        Commands.waitUntil(
            () -> modules[0].getCurrentCommand() == null
                && modules[1].getCurrentCommand() == null
                && modules[2].getCurrentCommand() == null
                && modules[3].getCurrentCommand() == null),
        Commands.runOnce(() -> driveFieldRelative(new ChassisSpeeds(0, 0, 0.5))),
        Commands.waitSeconds(2.0),
        Commands.runOnce(
            () -> {
              driveFieldRelative(new ChassisSpeeds());
              if (-imu.getRate() < Units.radiansToDegrees(0.3)) {
                addFault("[System Check] IMU rate too low", false, true);
              }
            },
            this),
        Commands.runOnce(() -> driveFieldRelative(new ChassisSpeeds(0, 0, -0.5))),
        Commands.waitSeconds(2.0),
        Commands.runOnce(
            () -> {
              driveFieldRelative(new ChassisSpeeds());
              if (-imu.getRate() > Units.radiansToDegrees(-0.3)) {
                addFault("[System Check] IMU rate too low", false, true);
              }
            },
            this))
        .until(
            () -> !getFaults().isEmpty()
                || !modules[0].getFaults().isEmpty()
                || !modules[1].getFaults().isEmpty()
                || !modules[2].getFaults().isEmpty()
                || !modules[3].getFaults().isEmpty())
        .andThen(Commands.runOnce(() -> driveFieldRelative(new ChassisSpeeds()), this));
  }

  /*
   * @Override
   * public SystemStatus getSystemStatus() {
   * SystemStatus worstStatus = SystemStatus.OK;
   * 
   * for (SubsystemFault f : this.getFaults()) {
   * if (f.sticky || f.timestamp > Timer.getFPGATimestamp() - 10) {
   * if (f.isWarning) {
   * if (worstStatus != SystemStatus.ERROR) {
   * worstStatus = SystemStatus.WARNING;
   * }
   * } else {
   * worstStatus = SystemStatus.ERROR;
   * }
   * }
   * }
   * 
   * for (Mk4SwerveModulePro module : modules) {
   * SystemStatus moduleStatus = module.getSystemStatus();
   * if (moduleStatus == SystemStatus.ERROR) {
   * worstStatus = SystemStatus.ERROR;
   * } else if (moduleStatus == SystemStatus.WARNING && worstStatus ==
   * SystemStatus.OK) {
   * worstStatus = SystemStatus.WARNING;
   * }
   * }
   * 
   * return worstStatus;
   * }
   */
}
