package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;

import static edu.wpi.first.units.Units.Rotation;

import java.util.concurrent.TimeUnit;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.subsystem.AdvancedSubsystem;

public class CoralHandlerWrist extends AdvancedSubsystem {

    private final String name;
    private final double gearRatio;
    private final Rotation2d armMinRotation;
    private final Rotation2d armMaxRotation;
    private final double minVelocity;

    private final SparkMax motor;
    private final SparkClosedLoopController controller;
    private final CANcoder absoluteEncoder;
    private final StatusSignal<Angle> absoluteSignal;

    private final RelativeEncoder relativeEncoder;

    public final DCMotor gearboxSim;
    public final SparkMaxSim motorSim;
    private final SingleJointedArmSim coralHandlerPhysicsSim;

    public CoralHandlerWrist(
            String name, // "Horizontal" or "Vertical"
            int motorId,
            int encoderId,
            double gearRatio,
            double p,
            double i,
            double d,
            double ff,
            double izone,
            double minVelocity,
            double maxVelocity,
            double maxAcceleration,
            double allowedError,
            LimitSwitchConfig.Type limitSwitchType,
            Rotation2d armMinRotation, // min rotation of the arm
            Rotation2d armMaxRotation, // max rotation of the arm
            double jKgMetersSquared,
            double coralEndEffectorLength,
            Rotation2d startingAngle) {
        super("CoralHandlerWrist" + name);
        this.name = name;
        this.gearRatio = gearRatio;
        this.minVelocity = minVelocity;
        this.armMinRotation = armMinRotation;
        this.armMaxRotation = armMaxRotation;

        this.motor = new SparkMax(motorId, SparkLowLevel.MotorType.kBrushless);
        this.controller = motor.getClosedLoopController();
        this.relativeEncoder = motor.getEncoder();
        this.relativeEncoder.setPosition(startingAngle.getRotations());

        LimitSwitchConfig limitConfig = new LimitSwitchConfig();
        limitConfig.forwardLimitSwitchType(limitSwitchType);
        SoftLimitConfig softLimitConfig = new SoftLimitConfig();
        softLimitConfig.forwardSoftLimitEnabled(false);
        softLimitConfig.reverseSoftLimitEnabled(false);
        // softLimitConfig.reverseSoftLimit(clampMin / rotationDegreesPerRotation);
        // softLimitConfig.forwardSoftLimit(clampMax / rotationDegreesPerRotation);

        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.inverted(false);
        motorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);

        // configure motor
        ClosedLoopConfig pidConfig = motorConfig.closedLoop;
        pidConfig.pidf(p, i, d, 0);
        pidConfig.iZone(izone);
        pidConfig.maxMotion.maxAcceleration(maxAcceleration);
        pidConfig.maxMotion.maxVelocity(maxVelocity);
        pidConfig.maxMotion.allowedClosedLoopError(allowedError);
        motorConfig.apply(pidConfig);
        motor.configure(motorConfig, SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters);

        // configure encoder
        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        encoderConfig.MagnetSensor.MagnetOffset = Preferences.getDouble(name.toLowerCase() + "RotationalOffset", 0);
        this.absoluteEncoder = new CANcoder(encoderId);
        absoluteEncoder.getConfigurator().apply(encoderConfig);
        absoluteEncoder.getConfigurator().setPosition(startingAngle.getRotations());
        this.absoluteSignal = absoluteEncoder.getAbsolutePosition();
        absoluteSignal.refresh();
            
        this.gearboxSim = DCMotor.getNeo550(1);

        coralHandlerPhysicsSim = new SingleJointedArmSim(gearboxSim,
                gearRatio, jKgMetersSquared, coralEndEffectorLength,
                armMinRotation.getRadians(), armMaxRotation.getRadians(),
                true, // gravity
                startingAngle.getRadians()); // ,Constants.CoralHandler.horizontalMotorStdDev);

        this.motorSim = new SparkMaxSim(motor, gearboxSim);

        registerHardware("Coral " + name + " Motor", motor);
        registerHardware("Coral " + name + " Encoder", absoluteEncoder);
    }

    @Override
    public void periodic() {
        absoluteSignal.refresh();
    }

    private long timeSinceLastLog = System.nanoTime();

    @Override
    public void simulationPeriodic() {
        double inputVoltage = motor.getAppliedOutput() * RoboRioSim.getVInVoltage();

        var initialArmPos = Rotation2d.fromRadians(coralHandlerPhysicsSim.getAngleRads());

        coralHandlerPhysicsSim.setInput(inputVoltage);
        coralHandlerPhysicsSim.update(0.02);

        var finalArmPos = Rotation2d.fromRadians(coralHandlerPhysicsSim.getAngleRads());
        var armRotation = finalArmPos.minus(initialArmPos);
        var motorRotation = armRotation.times(gearRatio);
        var motorRpm = motorRotation.getRotations() * (60 / .02);

        motorSim.iterate(motorRpm, RoboRioSim.getVInVoltage(), 0.02);

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(motorSim.getMotorCurrent()));

        if ( System.nanoTime() - timeSinceLastLog > TimeUnit.SECONDS.toNanos(1)) {
            System.out.printf("[%s] arm %.0f deg, sim arm %.0f deg, motor %.1f degrees, inputVoltage: %.1f V motorVelocity: %.0f rpm%n",
                    name,
                    Rotation2d.fromRotations(relativeEncoder.getPosition()).div(gearRatio).getDegrees(),
                    Rotation2d.fromRadians(coralHandlerPhysicsSim.getAngleRads()).getDegrees(), // arm degrees
                    Rotation2d.fromRotations(relativeEncoder.getPosition()).getDegrees(), // motor rotations
                    inputVoltage, motorRpm
            );
            timeSinceLastLog = System.nanoTime();
        }
    }

    /**
     * Sets the angle.
     *
     * @param targetArmAngle The needed angle to be obtained.
     */
    public void setAngle(Rotation2d targetArmAngle) {
        // // clamp target arm angle
        // targetArmAngle = Rotation2d.fromDegrees(
        //         MathUtil.clamp(targetArmAngle.getDegrees(), armMinRotation.getDegrees(), armMaxRotation.getDegrees()));

        var targetMotorAngle = targetArmAngle.times(gearRatio);

        System.out.printf("[%s] set arm target to %.0f degrees%n", name, targetArmAngle.getDegrees());

        controller.setReference(targetMotorAngle.getRotations(), SparkBase.ControlType.kPosition);
        timeSinceLastLog = System.nanoTime();
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(relativeEncoder.getPosition()).div(gearRatio);
    }
    public void stopMotor() {
        motor.stopMotor();
    }

    public Command setAngleCommand(Rotation2d targetAngle) {
        SmartDashboard.putNumber(name + "Target Angle in Degrees", targetAngle.getDegrees());
        return Commands.sequence(
                Commands.runOnce(() -> setAngle(targetAngle), this),
                Commands.waitUntil(() -> {
                    var difference = Math.abs(Rotation2d.fromRotations(relativeEncoder.getPosition()).minus(targetAngle).getDegrees());
                    System.out.printf("[%s] difference=%.0f degrees%n", name, difference);
                    return difference < 5;
                }),
                Commands.runOnce(() -> {
                    System.out.printf("[%s] got within 5 degrees", name);
                    motor.stopMotor();
                }, this));
    }

    public void putAbsoluteEncoderSimAngle() {
        SmartDashboard.putNumber(name + " Absolute Encoder Sim Angle",
                motorSim.getAbsoluteEncoderSim().getPosition());
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
                            if (Math.abs(getAngle().getDegrees() - Rotation2d.fromDegrees(45).getDegrees()) < 0.5) {
                                addFault("[System Check] " + name + " Coral Motor did not get to target angle (45 degrees)",
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
                            if (Math.abs(getAngle().getDegrees() - Rotation2d.fromDegrees(-45).getDegrees()) < 0.5) {
                                addFault("[System Check] " + name + " Coral Motor did not get to target angle (-45 degrees)",
                                false, true);
                            }
                            motor.stopMotor();
                        }, this));
    }
}
