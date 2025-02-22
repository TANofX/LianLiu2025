package frc.lib.subsystem.selfcheck;

import java.util.ArrayList;
import java.util.List;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;

import frc.lib.subsystem.SubsystemFault;

public class SelfCheckingSparkBase implements SelfChecking {
  private final String label;
  private final SparkBase spark;

  public SelfCheckingSparkBase(String label, SparkBase spark) {
    this.label = label;
    this.spark = spark;
  }

  @Override
  public List<SubsystemFault> checkForFaults() {
    ArrayList<SubsystemFault> faults = new ArrayList<>();

    REVLibError err = spark.getLastError();
    if (err != REVLibError.kOk) {
      faults.add(new SubsystemFault(String.format("[%s]: Error: %s", label, err.name())));
    }

    return faults;
  }
}
