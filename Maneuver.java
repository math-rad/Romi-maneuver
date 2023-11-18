// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import java.io.Console;
import java.lang.ModuleLayer.Controller;
import java.time.OffsetDateTime;
import java.util.function.Supplier;
import java.util.function.ToIntFunction;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;

public class Maneuver extends CommandBase {
  private final Drivetrain m_drivetrain;
  private final Supplier<Double> control_XAxisSupplier;
  private final Supplier<Double> control_YAxisSupplier;
  private final CommandXboxController control;
  private final PIDController PIDControl = new PIDController(0.01, 0.0,  0.0 );
  private final double speedDampener = .75;
  private final Supplier<Boolean> leftDown;
  private double initXOffset;
  private double initYOffset;
  private double initAngle;
  private double activeOffset;

  /**
   * Creates a new ArcadeDrive. This command will drive your robot according to
   * the speed supplier
   * lambdas. This command does not terminate.
   *
   * @param drivetrain          The drivetrain subsystem on which this command
   *                            will run
   * @param xaxisSpeedSupplier  Lambda supplier of forward/backward speed
   * @param yaxisRotateSupplier Lambda supplier of rotational speed
   * @param leftButton          Left joystick down
   */
  public Maneuver(
      Drivetrain drivetrain,
      CommandXboxController commandXbox) {
    m_drivetrain = drivetrain;
    control = commandXbox;

    control_XAxisSupplier = () -> control.getLeftX();
    control_YAxisSupplier = () -> control.getLeftY();
    leftDown = () -> control.button(9).getAsBoolean();

    initXOffset = control_XAxisSupplier.get();
    initYOffset = control_YAxisSupplier.get();
    
    var point = new Translation2d(control.getLeftX(), control.getLeftY());
    initAngle = point.getAngle().getDegrees();
    this.PIDControl.setTolerance(2);
    PIDControl.enableContinuousInput(-180, 180);
    


    
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (control.a().getAsBoolean()) {
      m_drivetrain.resetGyro();
    }

    double leftX = control_XAxisSupplier.get() - initXOffset;
    double leftY = control_YAxisSupplier.get() - initYOffset;

    var point = new Translation2d(leftX, leftY);

    double leftAngle = point.getAngle().getDegrees(); // + activeOffset
    double leftNormal = point.getNorm();
    
    double zOrientation = Math.IEEEremainder(m_drivetrain.getGyroAngleZ(), 360);

    double speed = 0;
    double rotation = 0;
    System.out.print(leftAngle);
    System.out.println();
    PIDControl.setSetpoint(leftAngle);

    speed = leftNormal * speedDampener * (leftDown.get() ? -1 : 1);

    var potentialRotation = -PIDControl.calculate(zOrientation);

    rotation = (PIDControl.atSetpoint() ? 0 : potentialRotation);
    
    m_drivetrain.arcadeDrive(speed, rotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // This is a default command
  }
}
