package frc.lib.input.controllers.rumble;

import java.util.Objects;

public class RumblePulse extends RumbleAnimation {
  private final double pulsePeriod;
  private final double intensity;

  public RumblePulse(double pulsesPerSecond, double intensity) {
    this.pulsePeriod = 1.0 / pulsesPerSecond;
    this.intensity = intensity;
  }

  public RumblePulse(double pulsesPerSecond) {
    this(pulsesPerSecond, 1.0);
  }

  @Override
  public double getRumbleOutput(double timeSeconds) {
    if (timeSeconds % pulsePeriod < pulsePeriod / 2) {
      return 0;
    } else {
      return intensity;
    }
  }

  @Override
  public boolean equals(Object o) {
    if (this == o) return true;
    if (o == null || getClass() != o.getClass()) return false;
    RumblePulse that = (RumblePulse) o;
    return Double.compare(that.pulsePeriod, pulsePeriod) == 0
        && Double.compare(that.intensity, intensity) == 0;
  }

  @Override
  public int hashCode() {
    return Objects.hash(pulsePeriod, intensity);
  }
}
