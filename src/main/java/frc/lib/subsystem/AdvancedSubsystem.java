package frc.lib.subsystem;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.subsystem.selfcheck.SelfChecking;
import frc.lib.subsystem.selfcheck.SelfCheckingCANCoderPro;
import frc.lib.subsystem.selfcheck.SelfCheckingPigeon2;
import frc.lib.subsystem.selfcheck.SelfCheckingSparkBase;
import frc.lib.subsystem.selfcheck.SelfCheckingTalonFXPro;

public abstract class AdvancedSubsystem extends SubsystemBase {
  public enum SystemStatus {
    OK,
    WARNING,
    ERROR
  }

  private final List<SubsystemFault> faults = new ArrayList<>();
  private final List<SelfChecking> hardware = new ArrayList<>();
  private final String statusTable;
  private final boolean checkErrors;
  private final Timer faultsTimer = new Timer();
  private final Timer statusTimer = new Timer();
  
  public AdvancedSubsystem() {
    this("");
  }

  public AdvancedSubsystem(String name) {
    checkErrors = RobotBase.isReal();
    initializeTimers();
    // setupCallbacks();
    this.setName(name);
    this.statusTable = "SystemStatus/" + this.getName();
  }

  public void registerSystemCheckWithSmartDashboard() {
    Command systemCheck = getSystemCheckCommand();
    systemCheck.setName(getName() + "Check");
    SmartDashboard.putData(statusTable + "/SystemCheck", systemCheck);
    SmartDashboard.putBoolean(statusTable + "/CheckRan", false);
  }

  private void initializeTimers() {
    faultsTimer.start();
    statusTimer.start();
  }

  public Command getSystemCheckCommand() {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              SmartDashboard.putBoolean(statusTable + "/CheckRan", false);
              clearFaults();
              publishStatus();
            }),
        systemCheckCommand(),
        Commands.runOnce(
            () -> {
              publishStatus();
              SmartDashboard.putBoolean(statusTable + "/CheckRan", true);
            }));
  }

  @Override
  public void periodic() {
    if (faultsTimer.advanceIfElapsed(0.25)) checkForFaults();
    if (statusTimer.advanceIfElapsed(1.0)) publishStatus();
  }

  // private void setupCallbacks() {
  //   Robot.addPeriodic(this::checkForFaults, 0.25);
  //   Robot.addPeriodic(this::publishStatus, 1.0);
  // }

  private void publishStatus() {
    SystemStatus status = getSystemStatus();
    SmartDashboard.putString(statusTable + "/Status", status.name());
    SmartDashboard.putBoolean(statusTable + "/SystemOK", status == SystemStatus.OK);

    String[] faultStrings = new String[this.faults.size()];
    for (int i = 0; i < this.faults.size(); i++) {
      SubsystemFault fault = this.faults.get(i);
      faultStrings[i] = String.format("[%.2f] %s", fault.timestamp, fault.description);
    }
    SmartDashboard.putStringArray(statusTable + "/Faults", faultStrings);

    if (faultStrings.length > 0) {
      SmartDashboard.putString(statusTable + "/LastFault", faultStrings[faultStrings.length - 1]);
    } else {
      SmartDashboard.putString(statusTable + "/LastFault", "");
    }
  }

  protected void reportPowerUsage(String device, double amps, double volts) {
    //    BatteryUsage.reportUsage(this.getName(), device, amps, volts);
  }

  protected void addFault(SubsystemFault fault) {
    if (!this.faults.contains(fault)) {
      this.faults.add(fault);
    }
  }

  protected void addFault(String description, boolean isWarning) {
    this.addFault(new SubsystemFault(description, isWarning));
  }

  protected void addFault(String description, boolean isWarning, boolean sticky) {
    this.addFault(new SubsystemFault(description, isWarning, sticky));
  }

  protected void addFault(String description) {
    this.addFault(description, false);
  }

  public List<SubsystemFault> getFaults() {
    return this.faults;
  }

  public void clearFaults() {
    this.faults.clear();
  }

  public SystemStatus getSystemStatus() {
    SystemStatus worstStatus = SystemStatus.OK;

    for (SubsystemFault f : this.faults) {
      if (f.sticky || f.timestamp > Timer.getFPGATimestamp() - 10) {
        if (f.isWarning) {
          if (worstStatus != SystemStatus.ERROR) {
            worstStatus = SystemStatus.WARNING;
          }
        } else {
          worstStatus = SystemStatus.ERROR;
        }
      }
    }
    return worstStatus;
  }

 // public void registerHardware(String label, BaseMotorController phoenixMotor) {
 //   hardware.add(new SelfCheckingPhoenixMotor(label, phoenixMotor));
  //}

  public void registerHardware(String label, TalonFX driveMotor) {
    hardware.add(new SelfCheckingTalonFXPro(label, driveMotor));
  }


  public void registerHardware(String label, SparkBase spark) {
    hardware.add(new SelfCheckingSparkBase(label, spark));
  }

  public void registerHardware(String label, com.ctre.phoenix6.hardware.Pigeon2 pigeon2) {
    hardware.add(new SelfCheckingPigeon2(label, pigeon2));
  }


  //public void registerHardware(String label, CANCoder canCoder) {
    //hardware.add(new SelfCheckingCANCoder(label, canCoder));
  //}

  public void registerHardware(String label, CANcoder canCoder) {
    hardware.add(new SelfCheckingCANCoderPro(label, canCoder));
  }

  // Command to run a full systems check
  protected abstract Command systemCheckCommand();

  // Method to check for faults while the robot is operating normally
  private void checkForFaults() {
    if (checkErrors) {
      for (SelfChecking device : hardware) {
        for (SubsystemFault fault : device.checkForFaults()) {
          addFault(fault);
        }
      }
    }
  }
}
