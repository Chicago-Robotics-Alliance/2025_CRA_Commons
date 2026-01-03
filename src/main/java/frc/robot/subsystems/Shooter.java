package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.VoltsPerRadianPerSecond;
import static yams.mechanisms.SmartMechanism.gearbox;
import static yams.mechanisms.SmartMechanism.gearing;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import CRA.FeedForwardTuning;
import CRA.PIDTuning;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class Shooter extends SubsystemBase {

  public GenericEntry shooterVoltageSet;
  private SimpleMotorFeedforward shooterFeedforward;
  private FeedForwardTuning shooterFeedForwardTuning;
  private PIDController shooterPID;
  private PIDTuning shooterPIDtuning;
  private ShuffleboardTab tab;
  // TODO: Add detailed comments explaining the example, similar to the
  // ExponentiallyProfiledArmSubsystem

  SmartMotorControllerConfig smcConfig;

  // Vendor motor controller object
  SparkFlex topShooter;
  SparkFlex bottomShooter;

  SparkFlexConfig topShooterConfig;
  SparkFlexConfig bottomShooterConfig;

  // Create our SmartMotorController from our Spark and config with the NEO.
  SmartMotorController sparkSmartMotorController;
  FlyWheelConfig shooterConfig;

  // Shooter Mechanism
  private FlyWheel shooter;

  public Shooter() {

    tab = Shuffleboard.getTab(getName());
    shooterFeedforward = new SimpleMotorFeedforward(0, 0.109, 0);
    shooterFeedForwardTuning = new FeedForwardTuning(getName(), shooterFeedforward.getKs(), 0,
        shooterFeedforward.getKv(), shooterFeedforward.getKa());
    shooterPID = new PIDController(0.0, 0, 0);
    shooterPIDtuning = new PIDTuning(getName(), shooterPID.getP(), shooterPID.getI(), shooterPID.getD(), 0);

    smcConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        // Feedback Constants (PID Constants)
        .withClosedLoopController(shooterPID.getP(), shooterPID.getI(), shooterPID.getD(), DegreesPerSecond.of(90),
            DegreesPerSecondPerSecond.of(45))
        .withSimClosedLoopController(0, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
        // Feedforward Constants
        .withFeedforward(shooterFeedforward)
        .withSimFeedforward(shooterFeedforward)
        // Telemetry name and verbosity level
        .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
        // Gearing from the motor rotor to final shaft.
        // In this example gearbox(3,4) is the same as gearbox("3:1","4:1") which
        // corresponds to the gearbox attached to your motor.
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
        // Motor properties to prevent over currenting.
        .withMotorInverted(false)
        .withIdleMode(MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(40))
        .withClosedLoopRampRate(Seconds.of(0.25))
        .withOpenLoopRampRate(Seconds.of(0.25));

    // Vendor motor controller object
    topShooter = new SparkFlex(42, MotorType.kBrushless);
    topShooterConfig = new SparkFlexConfig();
    topShooterConfig.softLimit.forwardSoftLimitEnabled(true);
    topShooterConfig.softLimit.forwardSoftLimit(30);
    topShooterConfig.softLimit.reverseSoftLimit(30);
    topShooterConfig.softLimit.reverseSoftLimitEnabled(true);
    topShooterConfig.idleMode(IdleMode.kCoast);
    topShooter.configure(topShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    bottomShooter = new SparkFlex(43, MotorType.kBrushless);

    bottomShooterConfig = new SparkFlexConfig();
    bottomShooterConfig.softLimit.forwardSoftLimitEnabled(true);
    bottomShooterConfig.softLimit.forwardSoftLimit(30);
    bottomShooterConfig.softLimit.reverseSoftLimit(30);
    bottomShooterConfig.softLimit.reverseSoftLimitEnabled(true);
    bottomShooterConfig.follow(topShooter, true);
    bottomShooterConfig.idleMode(IdleMode.kCoast);
    bottomShooter.configure(bottomShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);



    // Create our SmartMotorController from our Spark and config with the NEO.
    sparkSmartMotorController = new SparkWrapper(topShooter, DCMotor.getNEO(1), smcConfig);
    // sparkSmartMotorController2 = new SparkWrapper(bottomShooter, DCMotor.getNEO(1), smcConfig2);

    shooterConfig = new FlyWheelConfig(sparkSmartMotorController)
        // Diameter of the flywheel.
        .withDiameter(Inches.of(4))
        // Mass of the flywheel.
        .withMass(Pounds.of(1))
        // Maximum speed of the shooter.
        .withUpperSoftLimit(RPM.of(1000))
        // Telemetry name and verbosity for the arm.
        .withTelemetry("Shooter", TelemetryVerbosity.HIGH);

    // Shooter Mechanism
    shooter = new FlyWheel(shooterConfig);
    shooterVoltageSet = tab.add(getName() + "shooterVoltageSet", shooter.getMotor().getVoltage().in(Volts)).getEntry();
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Shooter/velocity", shooter.getMotor().getMechanismVelocity());
    Logger.recordOutput("Shooter/volts", shooter.getMotor().getVoltage());
    Logger.recordOutput("Shooter/desiredvoltage", shooterVoltageSet.getDouble(0));
    Logger.recordOutput("Shooter/desiredvoltage", shooterVoltageSet.getDouble(0));
    shooter.updateTelemetry();
  }

  public Command shooterVoltageFromElastic() {
    return shooter.setVoltage(() -> Volts.of(shooterVoltageSet.getDouble(0)));
  }

  public Command shooterFeedForwardsFromElastic() {
    return this.runOnce(() -> {
      shooter.getMotorController().setFeedforward(
          shooterFeedForwardTuning.getFeedForward()[0],
          shooterFeedForwardTuning.getFeedForward()[2],
          shooterFeedForwardTuning.getFeedForward()[3],
          shooterFeedForwardTuning.getFeedForward()[1]);

      System.out.println("arm kS: " + shooterFeedforward.getKs());
      System.out.println("arm kV: " + shooterFeedforward.getKv());
      System.out.println("arm kA: " + shooterFeedforward.getKa());

      shooter.getMotorController().setFeedback(
          shooterPIDtuning.getPID()[0],
          shooterPIDtuning.getPID()[1],
          shooterPIDtuning.getPID()[2]);

      System.out.println("arm P: " + shooterPID.getP());
      System.out.println("arm I: " + shooterPID.getI());
      System.out.println("arm D: " + shooterPID.getD());
    });
  }

  public AngularVelocity getTopVelocity() {
    return shooter.getSpeed();
  }

  public AngularVelocity getBottomVelocity() {
    return AngularVelocity.ofBaseUnits(bottomShooter.getEncoder().getVelocity(), RotationsPerSecond);
  }

  public Command setVelocity(AngularVelocity speed) {
    return shooter.setSpeed(speed);
  }

  public Command setDutyCycle(double dutyCycle) {
    return shooter.set(dutyCycle);
  }

  public Command setVelocity(Supplier<AngularVelocity> speed) {
    return shooter.setSpeed(speed);
  }

  public Command setDutyCycle(Supplier<Double> dutyCycle) {
    return shooter.set(dutyCycle);
  }

  @Override
  public void simulationPeriodic() {
    shooter.simIterate();
  }

  public static final int ARM_PIVOT_LEFT_MOTOR = 40;
  public static final int ARM_PIVOT_RIGHT_MOTOR = 41;
  public static final double ARM_POSITION_TOLERANCE = 2.0;
  public static final double ARM_GEAR_RATIO = (1.0 / 125.0) * (24.0 / 58.0); // was (1.0/125.0)*(18.0/56.0)
  public static final double ARM_CONVERSION_REL_TO_ANGLE = 73 / 56.0; // Based on emperical evidence
}