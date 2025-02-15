package frc.robot.subsystems;

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
import com.revrobotics.spark.config.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import java.util.concurrent.TimeUnit;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.subsystem.AdvancedSubsystem;

public class CoralHandlerWrist extends AdvancedSubsystem {
    // Creation of needed variables
    private final String name;
    private final double gearRatio;
    // private final Rotation2d armMinRotation;
    // private final Rotation2d armMaxRotation;
    // private final double minVelocity;

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
            Rotation2d startingAngle) {
        super("CoralHandlerWrist" + name);
        this.name = name;
        this.gearRatio = gearRatio;
        // this.minVelocity = minVelocity;
        // this.armMinRotation = armMinRotation;
        // this.armMaxRotation = armMaxRotation;

        this.motor = new SparkMax(motorId, SparkLowLevel.MotorType.kBrushless);
        this.controller = motor.getClosedLoopController();
        this.relativeEncoder = motor.getEncoder();
        this.relativeEncoder.setPosition(startingAngle.getRotations());
        this.encoderConfig = new CANcoderConfiguration();


        LimitSwitchConfig limitConfig = new LimitSwitchConfig();
        limitConfig.forwardLimitSwitchType(limitSwitchType);
        
        //TODO Enable soft limit switch?
        SoftLimitConfig softLimitConfig = new SoftLimitConfig();
        softLimitConfig.forwardSoftLimitEnabled(false);
        softLimitConfig.reverseSoftLimitEnabled(false);
        // softLimitConfig.reverseSoftLimit(clampMin / rotationDegreesPerRotation);
        // softLimitConfig.forwardSoftLimit(clampMax / rotationDegreesPerRotation);

        //TODO should motorConfig configure the motor inverted?
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.inverted(false);
        motorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);

        // Configure the PID controls of motor
        ClosedLoopConfig pidConfig = motorConfig.closedLoop;
        // pidConfig.pidf(posP, posI, posD, 0, ClosedLoopSlot.kSlot0, maxPosP, maxPosI, maxPosD, maxPosff, ClosedLoopSlot.kSlot1);
        pidConfig
            .p(posP, ClosedLoopSlot.kSlot0)
            .d(posI, ClosedLoopSlot.kSlot0)
            .d(posD, ClosedLoopSlot.kSlot0)
            .velocityFF(posFF,ClosedLoopSlot.kSlot0)
            .p(maxPosP, ClosedLoopSlot.kSlot1)
            .i(maxPosI, ClosedLoopSlot.kSlot1)
            .d(maxPosD, ClosedLoopSlot.kSlot1)
            .velocityFF(maxPosFF, ClosedLoopSlot.kSlot1)
            .iZone(posIZone, ClosedLoopSlot.kSlot0)
            .iZone(maxPosIZone, ClosedLoopSlot.kSlot1);
        pidConfig.maxMotion.maxAcceleration(maxAcceleration);
        pidConfig.maxMotion.maxVelocity(maxVelocity);
        pidConfig.maxMotion.allowedClosedLoopError(allowedError);
        motorConfig.apply(pidConfig);
        motor.configure(motorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

        // Configure Encoder
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        encoderConfig.MagnetSensor.MagnetOffset = Preferences.getDouble(name.toLowerCase() + "RotationalOffset", 0);
        this.absoluteEncoder = new CANcoder(encoderId);
        absoluteEncoder.getConfigurator().apply(encoderConfig);
        absoluteEncoder.getConfigurator().setPosition(startingAngle.getRotations());
        this.absoluteSignal = absoluteEncoder.getAbsolutePosition();
        absoluteSignal.refresh();
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
        absoluteSignal.refresh();
    }
    
    //Time for logging on terminal
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
        if ( System.nanoTime() - timeSinceLastLog > TimeUnit.SECONDS.toNanos(1)) {
            System.out.printf("[%s] arm %.0f deg, sim arm %.0f deg, motor %.1f degrees, inputVoltage: %.1f V motorVelocity: %.0f rpm%n",
                    name,
                    Rotation2d.fromRotations(relativeEncoder.getPosition()).div(gearRatio).getDegrees(), //sim arm degrees
                    Rotation2d.fromRadians(coralHandlerPhysicsSim.getAngleRads()).getDegrees(), // arm degrees
                    Rotation2d.fromRotations(relativeEncoder.getPosition()).getDegrees(), // motor rotations
                    inputVoltage, motorRpm
            );
            timeSinceLastLog = System.nanoTime();
        }
    }

    public void runCoralWrist() {
        motor.set(.15);
    }
    
    /**
     * Sets the angle of coral handler wrist.
     *
     * @param targetArmAngle The needed angle to be obtained.
     */
    public void setAngle(Rotation2d targetArmAngle) {
        // // clamp target arm angle
        // targetArmAngle = Rotation2d.fromDegrees(
        //         MathUtil.clamp(targetArmAngle.getDegrees(), armMinRotation.getDegrees(), armMaxRotation.getDegrees()));
        var targetMotorAngle = targetArmAngle.times(gearRatio);

        System.out.printf("[%s] set arm target to %.0f degrees%n", name, targetArmAngle.getDegrees());
        
        //TODO check if this PID changing method works and change comment after + change angle for when PID controltype changes
        if ((targetMotorAngle.getDegrees() - getAngle().getDegrees()) > Rotation2d.fromDegrees(5).getDegrees())
            controller.setReference(targetMotorAngle.getRotations(), SparkBase.ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
        
        else {
            controller.setReference(targetMotorAngle.getRotations(), SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot1);
        }

        timeSinceLastLog = System.nanoTime();
    }

    /**
     * Gets the angle from the motor in a Rotation2d.
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(relativeEncoder.getPosition()).div(gearRatio);
    }
    
    /**
     * Stops the motor / Sets the speed of the motor to 0.
     */
    public void stopMotor() {
        motor.stopMotor();
    }
    
    /**
     * Syncs wrist Absolute encoder and Relative encoder together (based off of absolute encoder)
     */
    public void syncWristEncoder() {
        motor.getEncoder().setPosition(absoluteEncoder.getAbsolutePosition().getValueAsDouble() * gearRatio);
    }

    /**
     * Changes where the offset of the absolute encoder is and syncs relative encoder to it
     */
    public void updateWristOffset() {
        double currentOffset = encoderConfig.MagnetSensor.MagnetOffset; //of absolute encoder
        double offset = (currentOffset - absoluteEncoder.getAbsolutePosition().getValueAsDouble()) % 1.0; //needed offset for absolute encoder
        Preferences.setDouble(getName() + "RotationOffset", offset * 360.0); 
        encoderConfig.MagnetSensor.MagnetOffset = offset; //makes it the new offset
        absoluteEncoder.getConfigurator().apply(encoderConfig); //applys offset
        syncWristEncoder(); //syncs absolute offset with relative
    }

    /**
     * Sets a target angle for the motor and runs the motor until the motor is close enough to the target angle (smaller than 5degrees). Stops motor once target angle is met.
     * @param targetAngle
     * @return Print statement that outputs the value of difference between the target angle and actual angle.
     */
    public Command setAngleCommand(Rotation2d targetAngle) {
        SmartDashboard.putNumber(name + "Target Angle in Degrees", targetAngle.getDegrees());
        return Commands.sequence(
                Commands.runOnce(() -> 
                    setAngle(targetAngle)),
                Commands.waitUntil(() -> {
                    var difference = Math.abs((Rotation2d.fromRotations(relativeEncoder.getPosition()).minus(targetAngle)).getDegrees());
                    System.out.printf("[%s] difference=%.0f degrees%n", name, difference);
                    return difference < 5;
                }),
                Commands.runOnce(() -> {
                    setAngle(targetAngle);
                    System.out.printf("[%s] got within 5 degrees", name);
                }, this));
    }

    /**
     * Outputs the absolute encoder simulation angle on SmartDashboard.
     */
    public void getAbsoluteEncoderSimAngle() {
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
                            if (Math.abs(getAngle().getDegrees() - Rotation2d.fromDegrees(45).getDegrees()) < 1) {
                                addFault("[System Check] " + name + " Coral Motor did not get to target angle (45 degrees) using MaxMotionPositionControl",
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
                                addFault("[System Check] " + name + " Coral Motor did not get to target angle (-45 degrees) using MaxMotionPositionControl",
                                false, true);
                            }
                            motor.stopMotor();
                        }, this));
    }
}
