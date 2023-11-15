// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;

public class Maneuver extends CommandBase {
  private final Drivetrain m_drivetrain;
  private final Supplier<Double> control_XAxisSupplier;
  private final Supplier<Double> control_YAxisSupplier;
  private final CommandXboxController control;
  private final PIDController PIDControl = new PIDController(0.2, 0.0,  0.0 );
  private final double speedDampener = 0.6;
  private final double maxAngleRange = 35;
  private final double leftControlAngleOffset = 45;
  private double activeOffset = 0;

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
    double leftX = control_XAxisSupplier.get();
    double leftY = control_YAxisSupplier.get();

    var point = new Translation2d(leftX, leftY);

    double leftAngle = point.getAngle().getDegrees() - leftControlAngleOffset;
    double leftNormal = point.getNorm();
    
    double zOrientation = (m_drivetrain.getGyroAngleZ() % 360) + activeOffset; // Oh so we don't start over after 360.. makes total sense to me

    double angleRange = Math.abs(zOrientation - leftAngle);

    double speed = 0;
    double rotation = 0;

    if (leftAngle == 0) {
      activeOffset = zOrientation;
    }


    PIDControl.setSetpoint(leftAngle);

    speed = leftNormal * speedDampener;

    if (angleRange >= maxAngleRange) {
      rotation = -PIDControl.calculate(zOrientation);
    }
    

    m_drivetrain.arcadeDrive(speed, rotation);

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // This is a default command
  }
}
