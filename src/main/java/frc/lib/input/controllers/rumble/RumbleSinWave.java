package frc.lib.input.controllers.rumble;

public class RumbleSinWave extends RumbleAnimation {
  private final double period;

  public RumbleSinWave(double periodSeconds) {
    this.period = periodSeconds;
  }

  @Override
  public double getRumbleOutput(double timeSeconds) {
    double t = (timeSeconds % period) / period;
    return (Math.sin(2 * Math.PI * t) + 1) / 2;
  }

  @Override
  public boolean equals(Object o) {
    if (this == o) return true;
    if (o == null || getClass() != o.getClass()) return false;
    RumbleSinWave that = (RumbleSinWave) o;
    return Double.compare(that.period, period) == 0;
  }

  @Override
  public int hashCode() {
    return getClass().hashCode();
  }
}
