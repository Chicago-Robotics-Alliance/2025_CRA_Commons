package CRA;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class PIDTuning {
    private double kP;
    private double kI;
    private double kD;
    private double kF;
    private String mechanism;
    private ShuffleboardTab tab;

    private GenericEntry kPEntry;
    private GenericEntry kIEntry; 
    private GenericEntry kDEntry;
    private GenericEntry kFEntry;

    public PIDTuning(String mechanism_name, double starting_kP, double starting_kI, double starting_kD, double starting_kF) {
        this.kP = starting_kP;
        this.kI = starting_kI;
        this.kD = starting_kD;
        this.kF = starting_kF;
        this.mechanism = mechanism_name;

        tab = Shuffleboard.getTab(mechanism);
    }

    public void onDashboard() {
        kPEntry = tab.add(mechanism + " kP", kP).getEntry();
        kIEntry = tab.add(mechanism + " kI", kI).getEntry();
        kDEntry = tab.add(mechanism + " kD", kD).getEntry();
        kFEntry = tab.add(mechanism + " kF", kF).getEntry();
    }

    public double[] getPID() {
        return new double[] {
            kPEntry.getDouble(kP), 
            kIEntry.getDouble(kI), 
            kDEntry.getDouble(kD), 
            kFEntry.getDouble(kF)
        };
    }
}
