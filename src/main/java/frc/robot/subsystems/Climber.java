package frc.robot.subsystems;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Climber extends SubsystemBase {
    private RelativeEncoder armEncoder;
    private double requestedPosition;
    private double climberDutyCycle;
    private PIDController climberPID;
    private final double gravityDutyCycle = 0.0;
    private SparkMax motor;
    private ArmFeedforward armFeedforward;
    private boolean initialized;

    public Climber() {
        initialized = false;
        motor = new SparkMax(Constants.Climber.MOTOR, SparkLowLevel.MotorType.kBrushless);
        SparkMaxConfig leverMotorConfig = new SparkMaxConfig();
        leverMotorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        leverMotorConfig.inverted(true);
        leverMotorConfig.encoder.positionConversionFactor(Constants.Climber.RADIANS_PER_REVOLUTION);
        leverMotorConfig.closedLoop.apply(Constants.Climber.CLOSED_LOOP_CONFIG);
        motor.configure(leverMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        armFeedforward = new ArmFeedforward(Constants.Climber.KS, Constants.Climber.KG, Constants.Climber.KV);
        armEncoder = motor.getEncoder();
        armFeedforward = new ArmFeedforward(Constants.Climber.KS, Constants.Climber.KG, Constants.Climber.KV);
    }

    public Command idle() {
        return this.runOnce(() -> {initialized = false; });
    }

    public void periodic(){
        SmartDashboard.putData("ClimberPID", climberPID);
        
        if (!initialized) {
            climberDutyCycle = 0.0;
            requestedPosition = armEncoder.getPosition();
            climberPID = new PIDController(2, 0, 0);
            climberPID.reset();
            initialized = true;
        }
        
        climberDutyCycle = climberPID.calculate(armEncoder.getPosition(), requestedPosition) + gravityDutyCycle;
        climberDutyCycle = MathUtil.clamp(climberDutyCycle, -0.5, 0.5);
        motor.set(climberDutyCycle);
    }

    public Command ascend(){
        return new InstantCommand(() -> {
            requestedPosition -= .03;
        }, this).repeatedly();
    }

    public Command descend(){
        return new InstantCommand(() -> {
            requestedPosition += .03;
        }, this).repeatedly();
    }

}
