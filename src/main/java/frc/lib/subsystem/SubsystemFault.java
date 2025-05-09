package frc.lib.subsystem;

import edu.wpi.first.wpilibj.Timer;

public class SubsystemFault {
  public final String description;
  public final double timestamp;
  public final boolean isWarning;
  public final boolean sticky;

  public SubsystemFault(String description, boolean isWarning) {
    this(description, isWarning, false);
  }

  public SubsystemFault(String description) {
    this(description, false);
  }

  public SubsystemFault(String description, boolean isWarning, boolean sticky) {
    this.description = description;
    this.timestamp = Timer.getFPGATimestamp();
    this.isWarning = isWarning;
    this.sticky = sticky;
  }

  @Override
  public boolean equals(Object other) {
    if (other == this) {
      return true;
    }

    if (other instanceof SubsystemFault o) {

      return description.equals(o.description) && isWarning == o.isWarning;
    }
    return false;
  }

  @Override
  public int hashCode() {
    int result = description.hashCode();
    result = 31 * result + (isWarning ? 1 : 0);
    return result;
  }
}
