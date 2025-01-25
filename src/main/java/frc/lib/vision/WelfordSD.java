package frc.lib.vision;

// Simple rolling Standard Deviations calculation using Welford's Algorithm
public class WelfordSD {
    private double mean = 0;
    private double m2 = 0;
    private int count = 0;
    private double changed = 0;
    private double previous = 0;

    public void addValue(double value) {
        changed = 5;
        count++;
        double delta = value - mean;
        mean += delta / count;
        m2 += delta * (value - mean); // Update sum of squared differences
    }

    public double getStandardDeviation() {
        if(changed < 0) return previous;
        changed--;
        return previous = count > 1 ? Math.sqrt(m2 / count) : 0;
    }

    public int getCount() {
        return count;
    }

    public void reset() {
        mean = 0;
        m2 = 0;
        count = 0;
        changed = 20;
    }
}

// Main concern, count might exceed int limit(impossible for current periodic frequency)