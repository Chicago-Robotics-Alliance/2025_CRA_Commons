package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static yams.mechanisms.SmartMechanism.gearbox;
import static yams.mechanisms.SmartMechanism.gearing;
import static edu.wpi.first.units.Units.VoltsPerRadianPerSecond;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import CRA.FeedForwardTuning;
import CRA.PIDTuning;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.gearing.Sprocket;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class ArmSubsystem extends SubsystemBase {
  public GenericEntry armVoltageSet;
  private ArmFeedforward armFeedforward;
  private FeedForwardTuning armFeedForwardTuning;
  private PIDController armPID;
  private PIDTuning armPIDtuning;
  private final SparkMax rightMotor = new SparkMax(41, MotorType.kBrushless);
  private final SparkMax leftMotor = new SparkMax(40, MotorType.kBrushless);
  private ShuffleboardTab tab;

  private final SmartMotorControllerConfig motorConfig;

  private final SmartMotorController motor;
  private final MechanismPositionConfig robotToMechanism;

  private ArmConfig m_config;
  private final Arm arm;

  // private final SmartMotorControllerTelemetryConfig motorTelemetryConfig = new
  // SmartMotorControllerTelemetryConfig()
  // .withMechanismPosition()
  // .withRotorPosition()
  // .withMechanismLowerLimit()
  // .withMechanismUpperLimit();

  public ArmSubsystem() {
    SparkMaxConfig armMotorConfig = new SparkMaxConfig();
    armMotorConfig.idleMode(SparkMaxConfig.IdleMode.kCoast);
    armMotorConfig.follow(40, true);
    rightMotor.configure(armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    tab = Shuffleboard.getTab(getName());
    armFeedforward = new ArmFeedforward(0.1, 0.36, 5.4, 0.2);
    armFeedForwardTuning = new FeedForwardTuning(getName(), armFeedforward.getKs(), armFeedforward.getKg(), armFeedforward.getKv(), armFeedforward.getKa());
    armPID = new PIDController(0.02, 0, 0);
    armPIDtuning = new PIDTuning(getName(), armPID.getP(), armPID.getI(), armPID.getD(), 0);

    motorConfig = new SmartMotorControllerConfig(this)
        .withClosedLoopController(armPID.getP(), armPID.getI(), armPID.getD(), DegreesPerSecond.of(180),
            DegreesPerSecondPerSecond.of(90))
        .withSoftLimit(Degrees.of(0), Degrees.of(80))
        .withGearing(new MechanismGearing(
            GearBox.fromReductionStages(5, 5, 5), Sprocket.fromStages("24:52")))
        // .withExternalEncoder(leftMotor.getAbsoluteEncoder())
        // .withZeroOffset(Rotations.of(0))
        .withIdleMode(MotorMode.COAST)
        .withTelemetry("leftMotor", TelemetryVerbosity.HIGH)
        // .withSpecificTelemetry("leftMotor", motorTelemetryConfig)
        .withStatorCurrentLimit(Amps.of(30))
        .withVoltageCompensation(Volts.of(12))
        .withMotorInverted(false)
        // .withClosedLoopRampRate(Seconds.of(0.25))
        // .withOpenLoopRampRate(Seconds.of(0.25)) turn ramp rate back on after test
        .withFeedforward(armFeedforward)
        .withControlMode(ControlMode.CLOSED_LOOP);
    motor = new SparkWrapper(leftMotor, DCMotor.getNEO(1), motorConfig);
    robotToMechanism = new MechanismPositionConfig()
        .withMaxRobotHeight(Meters.of(1.5))
        .withMaxRobotLength(Meters.of(0.75))
        .withRelativePosition(new Translation3d(Meters.of(0), Meters.of(0), Meters.of(0.5)));

    m_config = new ArmConfig(motor)
        .withLength(Meters.of(0.135))
        .withHardLimit(Degrees.of(0), Degrees.of(90))
        .withTelemetry("Arm", TelemetryVerbosity.HIGH)
        .withMass(Pounds.of(1))
        .withStartingPosition(Degrees.of(0))
        // .withHorizontalZero(Degrees.of(0))
        .withMechanismPositionConfig(robotToMechanism);
    arm = new Arm(m_config);

    armVoltageSet = tab.add(getName() + "armVoltageSet", arm.getMotor().getVoltage().in(Volts)).getEntry();
  }

  public void periodic() {
    Logger.recordOutput("Arm/position", arm.getMotor().getMechanismPosition());
    Logger.recordOutput("Arm/velocity", arm.getMotor().getMechanismVelocity());
    Logger.recordOutput("Arm/volts", arm.getMotor().getVoltage());
    Logger.recordOutput("Arm/desiredvoltage", armVoltageSet.getDouble(0));
    arm.updateTelemetry();
  }

  public void simulationPeriodic() {
    arm.simIterate();
  }

  public Command armCmdVoltage(double voltage) {
    return arm.setVoltage(Volts.of(voltage));
  }

  public Command armVoltageFromElastic() {
    return arm.setVoltage(() -> (Volts.of(0.5 * Math.cos(arm.getAngle().in(Radians)) + armVoltageSet.getDouble(0))));
  }

  public Command armFeedForwardsFromElastic() {
    return this.runOnce(() -> {
      arm.getMotorController().setFeedforward(
          armFeedForwardTuning.getFeedForward()[0],
          armFeedForwardTuning.getFeedForward()[2],
          armFeedForwardTuning.getFeedForward()[3],
          armFeedForwardTuning.getFeedForward()[1]
      );

      System.out.println("arm kS: " + armFeedforward.getKs());
      System.out.println("arm kV: " + armFeedforward.getKv());
      System.out.println("arm kA: " + armFeedforward.getKa());
      System.out.println("arm kG: " + armFeedforward.getKg());

      arm.getMotorController().setFeedback(
          armPIDtuning.getPID()[0],
          armPIDtuning.getPID()[1],
          armPIDtuning.getPID()[2]
      );

      System.out.println("arm P: " + armPID.getP());
      System.out.println("arm I: " + armPID.getI());
      System.out.println("arm D: " + armPID.getD());
    });
  }

  public Command armCmd(double dutycycle) {
    Logger.recordOutput("Arm/dc", dutycycle);
    return arm.set(dutycycle);
  }

  public Command sysId() {
    return arm.sysId(Volts.of(3), Volts.of(3).per(Second), Second.of(30));
  }

  public Command setAngle(Angle angle) {
    Logger.recordOutput("Arm/setAngle", angle);
    return arm.setAngle(angle);
  }
}