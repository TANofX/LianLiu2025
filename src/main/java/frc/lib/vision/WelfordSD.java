package frc.lib.vision;

/**
 * Simple rolling Standard Deviations calculation using Welford's Algorithm
 */
public class WelfordSD {
    private double mean = 0;
    private double m2 = 0;
    private int count = 0;
    private boolean changed = false;
    private double previous = 0;

    /**
     * Add a new value to the rolling standard deviation calculation.
     * @param value The new value
     */
    public void addValue(double value) {
        changed = true;
        count++;
        double delta = value - mean;
        mean += delta / count;
        m2 += delta * (value - mean); // Update sum of squared differences
    }

    /**
     * Get the latest standard deviation value.
     * Returns the same value if addValue have not been called for performance reasons.
     * @return The standard deviation
     */
    public double getStandardDeviation() {
        if(!changed) return previous;
        changed = false;
        return previous = count > 1 ? Math.sqrt(m2 / count) : 0;
    }

    /**
     * Get the count since the last reset.
     * @return The count
     */
    public int getCount() {
        return count;
    }

    /**
     * Reset the rolling standard deviation calculation.
     */
    public void reset() {
        mean = 0;
        m2 = 0;
        count = 0;
    }
}

// Main concern, count might exceed int limit(impossible for current periodic frequency)