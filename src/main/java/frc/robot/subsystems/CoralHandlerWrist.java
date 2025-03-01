package frc.robot.subsystems;

import java.util.concurrent.TimeUnit;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.subsystem.AdvancedSubsystem;

public final class CoralHandlerWrist extends AdvancedSubsystem {
    // Creation of needed variables
    private final String name;
    private final double gearRatio;

    private final SparkMax motor;
    private final SparkClosedLoopController controller;
    private final CANcoder absoluteEncoder;
    private final StatusSignal<Angle> absoluteSignal;

    private final RelativeEncoder relativeEncoder;

    public final DCMotor gearboxSim;
    public final SparkMaxSim motorSim;
    private final SingleJointedArmSim coralHandlerPhysicsSim;

    private final CANcoderConfiguration encoderConfig;
    
    // Creation of needed parameters for the Coral Handler Wrist
    public CoralHandlerWrist(
            String name, // "Horizontal" or "Vertical"
            int motorId,
            int encoderId,
            double gearRatio,
            double posP,
            double posI,
            double posD,
            double maxPosP,
            double maxPosI,
            double maxPosD,
            double posFF,
            double maxPosFF,
            double posIZone,
            double maxPosIZone,
            double minVelocity,
            double maxVelocity,
            double maxAcceleration,
            double allowedError,
            LimitSwitchConfig.Type limitSwitchType,
            Rotation2d armMinRotation,
            Rotation2d armMaxRotation,
            double jKgMetersSquared,
            double coralEndEffectorLength,
            Rotation2d startingAngle,
            Boolean motorInvert,
            SensorDirectionValue encoderDirection,
            Rotation2d softLimitForwardAngle,
            Rotation2d softLimitReverseAngle,
            double rotationDegreesPerRotation
            ) {
        super("CoralHandlerWrist" + name);
        this.name = name;
        this.gearRatio = gearRatio;

        this.motor = new SparkMax(motorId, SparkLowLevel.MotorType.kBrushless);
        this.absoluteEncoder = new CANcoder(encoderId);
        this.controller = motor.getClosedLoopController();
        this.relativeEncoder = motor.getEncoder();
        this.encoderConfig = new CANcoderConfiguration();

        LimitSwitchConfig limitConfig = new LimitSwitchConfig();
        limitConfig.forwardLimitSwitchType(limitSwitchType);

        // TODO Enable soft limit switch?
        SoftLimitConfig softLimitConfig = new SoftLimitConfig();
        softLimitConfig.forwardSoftLimitEnabled(true);
        softLimitConfig.reverseSoftLimitEnabled(true);
        softLimitConfig.forwardSoftLimit(softLimitForwardAngle.getDegrees() / rotationDegreesPerRotation);
        softLimitConfig.forwardSoftLimit(softLimitReverseAngle.getDegrees() / rotationDegreesPerRotation);

        // softLimitConfig.reverseSoftLimit(clampMin / rotationDegreesPerRotation);
        // softLimitConfig.forwardSoftLimit(clampMax / rotationDegreesPerRotation);

        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.inverted(motorInvert);
        motorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);

        // Configure the PID controls of motor
        ClosedLoopConfig pidConfig = motorConfig.closedLoop;

        pidConfig
                .p(posP, ClosedLoopSlot.kSlot0)
                .i(posI, ClosedLoopSlot.kSlot0)
                .d(posD, ClosedLoopSlot.kSlot0)
                .velocityFF(posFF, ClosedLoopSlot.kSlot0)
                .p(maxPosP, ClosedLoopSlot.kSlot1)
                .i(maxPosI, ClosedLoopSlot.kSlot1)
                .d(maxPosD, ClosedLoopSlot.kSlot1)
                .velocityFF(maxPosFF, ClosedLoopSlot.kSlot1)
                .iZone(posIZone, ClosedLoopSlot.kSlot0)
                .iZone(maxPosIZone, ClosedLoopSlot.kSlot1);
        pidConfig.maxMotion.maxAcceleration(maxAcceleration, ClosedLoopSlot.kSlot1);
        pidConfig.maxMotion.maxVelocity(maxVelocity, ClosedLoopSlot.kSlot1);
        pidConfig.maxMotion.allowedClosedLoopError(allowedError, ClosedLoopSlot.kSlot1);
        motorConfig.apply(pidConfig);
        motor.configure(motorConfig, SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters);

        // Configure Encoder
        encoderConfig.MagnetSensor.SensorDirection = encoderDirection;
        encoderConfig.MagnetSensor.MagnetOffset = Preferences.getDouble(name.toLowerCase() + "RotationOffset", 0) / 360.0;
        encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;

        absoluteEncoder.getConfigurator().apply(encoderConfig);
        this.absoluteSignal = absoluteEncoder.getAbsolutePosition();
        syncWristEncoder();

        this.gearboxSim = DCMotor.getNeo550(1);

        // Creation of Coral Handler Physics Simulation
        coralHandlerPhysicsSim = new SingleJointedArmSim(
                gearboxSim,
                gearRatio,
                jKgMetersSquared,
                coralEndEffectorLength,
                armMinRotation.getRadians(),
                armMaxRotation.getRadians(),
                true, // Gravity Boolean
                startingAngle.getRadians());
        // ,Constants.CoralHandler.horizontalMotorStdDev);

        this.motorSim = new SparkMaxSim(motor, gearboxSim);

        registerHardware("Coral " + name + " Motor", motor);
        registerHardware("Coral " + name + " Encoder", absoluteEncoder);
    }

    @Override
    public void periodic() {
        // Gets newest value of absolute encoder
        StatusSignal.waitForAll(
        0,
        absoluteSignal);
  }

    // Time for logging on terminal
    private long timeSinceLastLog = System.nanoTime();

    @Override
    public void simulationPeriodic() {
        // Voltage that is sent into the simulation motor
        double inputVoltage = motor.getAppliedOutput() * RoboRioSim.getVInVoltage();

        var initialArmPos = Rotation2d.fromRadians(coralHandlerPhysicsSim.getAngleRads());

        coralHandlerPhysicsSim.setInput(inputVoltage);
        // Update the physics simulation for amount of seconds
        coralHandlerPhysicsSim.update(0.02);

        var finalArmPos = Rotation2d.fromRadians(coralHandlerPhysicsSim.getAngleRads());
        var armRotation = finalArmPos.minus(initialArmPos);
        var motorRotation = armRotation.times(gearRatio);
        var motorRpm = motorRotation.getRotations() * (60 / .02);

        // Update the motor simulation using the found RPM of the motor
        motorSim.iterate(motorRpm, RoboRioSim.getVInVoltage(), 0.02);

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(motorSim.getMotorCurrent()));

        // Logging in Terminal
        if (System.nanoTime() - timeSinceLastLog > TimeUnit.SECONDS.toNanos(1)) {
            System.out.printf(
                    "[%s] arm %.0f deg, sim arm %.0f deg, motor %.1f degrees, inputVoltage: %.1f V motorVelocity: %.0f rpm%n",
                    name,
                    Rotation2d.fromRotations(relativeEncoder.getPosition()).div(gearRatio).getDegrees(), // sim arm
                                                                                                         // degrees
                    Rotation2d.fromRadians(coralHandlerPhysicsSim.getAngleRads()).getDegrees(), // arm degrees
                    Rotation2d.fromRotations(relativeEncoder.getPosition()).getDegrees(), // motor rotations
                    inputVoltage, motorRpm);
            timeSinceLastLog = System.nanoTime();
        }
    }

    public void runCoralWrist(double speed) {
        motor.set(speed);
    }

    /**
     * Sets the angle of coral handler wrist.
     *
     * @param targetArmAngle The needed angle to be obtained.
     */
    public void setAngle(Rotation2d targetArmAngle) {
        // // clamp target arm angle
        // targetArmAngle = Rotation2d.fromDegrees(
        // MathUtil.clamp(targetArmAngle.getDegrees(), armMinRotation.getDegrees(),
        // armMaxRotation.getDegrees()));
        var targetMotorAngle = targetArmAngle.times(gearRatio);

        // System.out.printf("[%s] set arm target to %.0f degrees%n", name, targetArmAngle.getDegrees());

        // TODO Check if this PID changing method works and change comment after +
        // change angle for when PID controlType changes
        // if ((Math.abs(targetMotorAngle.getDegrees() - getAngle().getDegrees())) > Rotation2d.fromDegrees(.5).getDegrees())
        //     controller.setReference(targetMotorAngle.getRotations(), SparkBase.ControlType.kMAXMotionPositionControl,
        //             ClosedLoopSlot.kSlot1);

        // else {
        //     controller.setReference(targetMotorAngle.getRotations(), SparkBase.ControlType.kPosition,
        //             ClosedLoopSlot.kSlot0);
        // }
        controller.setReference(targetMotorAngle.getRotations(), SparkBase.ControlType.kPosition,
                ClosedLoopSlot.kSlot0);
                
        timeSinceLastLog = System.nanoTime();
    }

    public Rotation2d getMotorRotations() {
        return Rotation2d.fromRotations(relativeEncoder.getPosition());
    }
    /**
     * Gets the angle from the motor in a Rotation2d from relative encoder.
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(relativeEncoder.getPosition()).div(gearRatio);
    }

    /**
     * Gets the angle from the motor in a Rotation2d from relative encoder.
     */
    public Rotation2d getAbsoluteAngle() {
        return Rotation2d.fromDegrees(absoluteSignal.getValueAsDouble() * 360);
    }

    public double getAppliedOutput() {
        return motor.getAppliedOutput();
    }
    /**
     * Stops the motor / Sets the speed of the motor to 0.
     */
    public void stopMotor() {
        motor.stopMotor();
    }

    /**
     * Syncs wrist Absolute encoder and Relative encoder together (based off of
     * absolute encoder)
     */
    public void syncWristEncoder() {
        absoluteSignal.waitForUpdate(1);
        motor.getEncoder().setPosition(absoluteSignal.getValueAsDouble() * gearRatio);
    }

    /**
     * Changes where the offset of the absolute encoder is and syncs relative
     * encoder to it
     */
    public void updateWristOffset() {
        double currentOffset = encoderConfig.MagnetSensor.MagnetOffset; // of absolute encoder
        absoluteSignal.refresh();
        double offset = (currentOffset - absoluteSignal.getValue().magnitude()) % 1.0; // needed offset for absolute encoder
        Preferences.setDouble(name.toLowerCase() + "RotationOffset", offset * 360.0);
        encoderConfig.MagnetSensor.MagnetOffset = offset; // makes it the new offset
        absoluteEncoder.getConfigurator().apply(encoderConfig); // applies offset
        syncWristEncoder(); // syncs absolute offset with relative
    }

    /**
     * Outputs the absolute encoder simulation angle on SmartDashboard.
     */
    public void getAbsoluteEncoderSimAngle() {
        SmartDashboard.putNumber(name + " Absolute Encoder Sim Angle",
                motorSim.getAbsoluteEncoderSim().getPosition());
    }

    /**
     * Sets a target angle for the motor and runs the motor until the motor is close
     * enough to the target angle (smaller than 5degrees). Stops motor once target
     * angle is met.
     * 
     * @param targetAngle
     * @return Print statement that outputs the value of difference between the
     *         target angle and actual angle.
     */
    public Command setAngleCommand(Rotation2d targetAngle) {
        // SmartDashboard.putNumber(name + "Target Angle in Degrees", targetAngle.getDegrees());
        return Commands.sequence(
                Commands.run(() -> { setAngle(targetAngle);
                }, this).until( () -> {
                    return Math.abs(targetAngle.getDegrees() - getAngle().getDegrees()) < 0.1;
                })).finallyDo(() -> {stopMotor();});
    }

    @Override
    public Command systemCheckCommand() {
        return Commands.sequence(
                Commands.runOnce(
                        () -> {
                            setAngle(Rotation2d.fromDegrees(45));
                        }, this),
                Commands.waitSeconds(3.0),
                Commands.runOnce(
                        () -> {
                            if (Math.abs(getAngle().getDegrees() - Rotation2d.fromDegrees(45).getDegrees()) < 1) {
                                addFault("[System Check] " + name
                                        + " Coral Motor did not get to target angle (45 degrees) using MaxMotionPositionControl",
                                        false, true);
                            }
                        }, this),
                Commands.runOnce(
                        () -> {
                            setAngle(Rotation2d.fromDegrees(-45));
                        }, this),
                Commands.waitSeconds(3.0),
                Commands.runOnce(
                        () -> {
                            if (Math.abs(getAngle().getDegrees() - Rotation2d.fromDegrees(-45).getDegrees()) < 1) {
                                addFault("[System Check] " + name
                                        + " Coral Motor did not get to target angle (-45 degrees) using MaxMotionPositionControl",
                                        false, true);
                            }
                            motor.stopMotor();
                        }, this));
    }
}
