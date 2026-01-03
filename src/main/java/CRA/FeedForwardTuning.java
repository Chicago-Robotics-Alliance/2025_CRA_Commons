package CRA;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
/**
 * FeedForwardTuning class for real-time FeedForward controller tuning on the Shuffleboard.
 * 
 * This class allows for live adjustment of the FeedForward parameters (kS, kG, kV, kA)
 * through the Shuffleboard interface, enabling quick tuning during robot operation.
 * Similar Usage as PIDTuning.
 * 
 * Usage:
 * - Create an instance of FeedForwardTuning with a unique mechanism name and initial FeedForward values.
 * - Use `getFeedForward()` to retrieve the current FeedForward values for use in your control logic.
 */
public class FeedForwardTuning {
    private double kS;
    private double kG;
    private double kV;
    private double kA;
    private String mechanism;
    private ShuffleboardTab tab;

    private GenericEntry kSEntry;
    private GenericEntry kGEntry;
    private GenericEntry kVEntry;
    private GenericEntry kAEntry;

    /**
     * Creates a new FeedForwardTuning instance for real-time FeedForward adjustment.
     * 
     * This constructor initializes a FeedForwardTuning interface for a specific mechanism,
     * setting up NetworkTables entries and starting values for live tuning during
     * robot testing.
     * 
     * @param mechanism_name Unique identifier for the mechanism being tuned (e.g., "shooter", "arm").
     *                      This name is used to create distinct NetworkTables entries and 
     *                      organize tuning parameters on the shuffleboard.
     * @param starting_kS Initial static voltage value. Identifies how much voltage is required to get the motor started.
     * @param starting_kG Initial gravitational voltage value. Identifies how much voltage is required to keep the mechanism stable when interacting with gravity.
     * @param starting_kV Initial velocity voltage value. Identifies how much voltage is required to keep the mechanism moving at a constant velocity.
     * @param starting_kA Initial acceleration voltage value. Identifies how much voltage is required to keep the mechanism accelerating at a constant acceleration.
     */
    public FeedForwardTuning(String mechanism_name, double starting_kS, double starting_kG, double starting_kV, double starting_kA) {
        this.kS = starting_kS;
        this.kG = starting_kG;
        this.kV = starting_kV;
        this.kA = starting_kA;
        this.mechanism = mechanism_name;

        tab = Shuffleboard.getTab(mechanism); // Creating the shuffleboard tab
        onDashboard(); // Initialize the entries on the Shuffleboard tab
    }

    /**
     * Initializes the FeedForwardTuning entries on the Shuffleboard dashboard. Runs internally.
     * 
     * This method creates and adds entries for kS, kG, kV, and kA to the specified
     * Shuffleboard tab, allowing for real-time adjustment of these parameters.
     */
    public void onDashboard() {
        kSEntry = tab.add(mechanism + " kS", kS).getEntry();
        kGEntry = tab.add(mechanism + " kG", kG).getEntry();
        kVEntry = tab.add(mechanism + " kV", kV).getEntry();
        kAEntry = tab.add(mechanism + " kA", kA).getEntry();
    }
    
    /**
     * Retrieves the current FeedForward values from the Shuffleboard entries.
     * 
     * This method reads the current values of kS, kG, kV, and kA from the
     * Shuffleboard entries and returns them as an array.
     * Call this in your periodic function and run .setFeedForward() with it.
     * 
     * @return An array containing the current FeedForward values in the order [kS, kG, kV, kA].
     */
    public double[] getFeedForward() {
        return new double[] {
            kSEntry.getDouble(kS), 
            kGEntry.getDouble(kG), 
            kVEntry.getDouble(kV), 
            kAEntry.getDouble(kA)
        };
    }
}
