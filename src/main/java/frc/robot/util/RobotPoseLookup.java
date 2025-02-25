package frc.robot.util;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;

public class RobotPoseLookup<T> {
  private final ArrayList<Timestamped<T>> prevPoses;
  private final double maxAge;

  public RobotPoseLookup() {
    this(0.5);
  }

  public RobotPoseLookup(double maxAgeSeconds) {
    prevPoses = new ArrayList<>();
    maxAge = maxAgeSeconds;
  }

  public void addPose(T pose) {
    double timestamp = Timer.getFPGATimestamp();

    prevPoses.add(new Timestamped<>(pose, timestamp));
    if (prevPoses.get(0).getTimestamp() < timestamp - maxAge) {
      prevPoses.remove(0);
    }
  }

  public T lookup(double timestamp) {
    Timestamped<T> closest = prevPoses.get(prevPoses.size() - 1);

    for (int i = prevPoses.size() - 2; i >= 0; i--) {
      if (Math.abs(prevPoses.get(i).getTimestamp() - timestamp)
          < Math.abs(closest.getTimestamp() - timestamp)) {
        closest = prevPoses.get(i);
      } else {
        break;
      }
    }

    return closest.getValue();
  }

  private class Timestamped<E> {
    private final E pose;
    private final double timestamp;

    private Timestamped(E inPose, double inTimestamp) {
      this.pose = inPose;
      this.timestamp = inTimestamp;
    }

    public E getValue() {
      return pose;
    }

    public double getTimestamp() {
      return timestamp;
    }
  }
}
