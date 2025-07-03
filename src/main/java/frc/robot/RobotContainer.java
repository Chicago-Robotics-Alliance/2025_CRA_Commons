// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

import swervelib.SwerveInputStream;

public class RobotContainer
{
  final CommandXboxController driverXbox = new CommandXboxController(0);
  //private final Sensation sensation = new Sensation();
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/robot"));
  // private final Conveyor conveyor = new Conveyor();
  // private final Lights lights = new Lights();
  // private final Climber climber = new Climber();
  private final SendableChooser<Command> autoChooser;
  private double wait_seconds = 5;

  //Trigger coralEnter = new Trigger(sensation::coralPresent);
 // Trigger coralHopper = new Trigger(sensation::coralInHopper);
 // Trigger coralExit = new Trigger(sensation::coralExitedHopper);

  //Driving the robot during teleOp
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
    drivebase.getSwerveDrive(),
    () -> driverXbox.getLeftY() * -1,
    () -> driverXbox.getLeftX() * -1)
    .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)  
    .deadband(OperatorConstants.DEADBAND)
    .scaleTranslation(0.8)  //might be changed to 1
    .allianceRelativeControl(true)
    .cubeRotationControllerAxis(true);

  //Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
    .withControllerHeadingAxis(() -> driverXbox.getRightX() * -1, () -> driverXbox.getRightY() * -1)
    .headingWhile(true);

   // Clone's the angular velocity input stream and converts it to a robotRelative input stream.
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy()
    .robotRelative(true)
    .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(
    drivebase.getSwerveDrive(),
    () -> -driverXbox.getLeftY(),
    () -> -driverXbox.getLeftX())
      .withControllerRotationAxis(() -> driverXbox.getRawAxis( 2))
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
    .withControllerHeadingAxis(
      () -> Math.sin(driverXbox.getRawAxis(2) * Math.PI) *(Math.PI *2),
      () -> Math.cos(driverXbox.getRawAxis(2) *Math.PI) *(Math.PI *2))
        .headingWhile(true)
        .translationHeadingOffset(true)
        .translationHeadingOffset(Rotation2d.fromDegrees( 0));

  public RobotContainer()
  {
    configureBindings();

    DriverStation.silenceJoystickConnectionWarning(true);
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putNumber("Wait Time", wait_seconds);
    NamedCommands.registerCommand("CustomWaitCommand", new WaitCommand(SmartDashboard.getNumber("Wait Time", wait_seconds)));
  }

  private void configureBindings()
  {
    Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
    //Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    //Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
    //Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    //Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    //Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleKeyboard);

    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } 
    else
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
    }

    //conveyor.setDefaultCommand(conveyor.clearCoral(coralHopper));
   // lights.setDefaultCommand(lights.set(Lights.Special.OFF));
   // climber.setDefaultCommand(climber.idle());

    if (Robot.isSimulation())
    {
      Pose2d target = new Pose2d(new Translation2d(1, 4), Rotation2d.fromDegrees(90));
      driveDirectAngleKeyboard.driveToPose(
        () -> target, 
        new ProfiledPIDController(5, 0,0, new Constraints(5, 2)),
        new ProfiledPIDController(5,0,0, new Constraints(Units.degreesToRadians(360), Units.degreesToRadians(180))));
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
      driverXbox.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
                                                      () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));
    }

    if (DriverStation.isTest())
    {
      //drivebase.setDefaultCommand(driveFieldOrienteAnglularVelocity); // Overrides drive command above!d
      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
    } 
    else
    {
      driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
      driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.rightBumper().onTrue(Commands.none());
     // driverXbox.povUp().whileTrue(climber.ascend());
     // driverXbox.povDown().whileTrue(climber.descend());
     // driverXbox.y().onTrue(lights.set(Lights.Special.RAINBOW));
     // driverXbox.b().onTrue(lights.set(Lights.Colors.WHITE, Lights.Patterns.MARCH));

      //coralEnter.and(coralExit.negate()).and(coralHopper.negate()).onTrue(lights.set(Lights.Colors.RED, Lights.Patterns.FAST_FLASH));
     // coralHopper.and(coralExit.negate()).onTrue(lights.set(Lights.Colors.RED, Lights.Patterns.MARCH));
     // coralExit.onFalse(lights.set(Lights.Colors.RED, Lights.Patterns.SOLID));
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
