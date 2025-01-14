// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.XboxController;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final XboxController controller = new XboxController(0);// we'll figure out a port
  private RobotContainer m_robotContainer;
  private final Drivetrain drivetrain = new Drivetrain();
  // slew rate limiter lowers the rate of change of the input directly to smoothen
  // movement
  private SlewRateLimiter smoothDrive = new SlewRateLimiter(0.5);
  private SwerveModule[] swerveModules = new SwerveModule[]{drivetrain.frontLeftSwerveModule,
                                                            drivetrain.frontRightSwerveModule,
                                                            drivetrain.backLeftSwerveModule,
                                                            drivetrain.backRightSwerveModule};


  private CANcoder frontLeft = new CANcoder(8);
  private CANcoder frontRight = new CANcoder(6);
  private CANcoder backLeft = new CANcoder(2);
  private CANcoder backRight = new CANcoder(4);

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    Preferences.initDouble("driveKP",1.0);
    Preferences.initDouble("driveKI",0.0);
    Preferences.initDouble("driveKD",0.0);
    Preferences.initDouble("turnKI",0.0);
    Preferences.initDouble("turnKI",0.0);
    Preferences.initDouble("turnKI",0.0);
    Preferences.initDouble("driveKS",0.0);
    Preferences.initDouble("driveKV",0.0);
    Preferences.initDouble("driveKA",0.0);
    Preferences.initDouble("turnKS",0.0);
    Preferences.initDouble("turnKV",0.0);
    Preferences.initDouble("turnKA",0.0);

    for (SwerveModule module : swerveModules){
      module.refreshTuning();
    }
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    if(controller.getAButtonPressed()){
      for (SwerveModule module : swerveModules){
        module.refreshTuning();
      }
    }
  }

  /** This function is called once each bbbhtime the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    System.out.println(frontLeft.getAbsolutePosition());
    System.out.println(frontRight.getAbsolutePosition());
    System.out.println(backLeft.getAbsolutePosition());
    System.out.println(backRight.getAbsolutePosition());
    driveWithJoystick();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }

  private void driveWithJoystick() {
    // double calculatedLeftX = smoothDrive.calculate(MathUtil.applyDeadband(-controller.getLeftX(), 0.02))
    //     * RobotConstants.MaxSpeed;
    // double calculatedLeftY = smoothDrive.calculate(MathUtil.applyDeadband(controller.getLeftY(), 0.02))
    //     * RobotConstants.MaxSpeed;
    // double calculatedRightY = smoothDrive.calculate(MathUtil.applyDeadband(-controller.getRightX(), 0.02))
    //     * RobotConstants.MaxAngularSpeed;

    double calculatedLeftX = MathUtil.applyDeadband(-controller.getLeftX(), 0.05)
        * RobotConstants.MaxSpeed;
    double calculatedLeftY = MathUtil.applyDeadband(controller.getLeftY(), 0.05)
        * RobotConstants.MaxSpeed;
    double calculatedRightY = MathUtil.applyDeadband(-controller.getRightX(), 0.05)
        * RobotConstants.MaxAngularSpeed;
    System.out.println("Drive Params: "+calculatedLeftX+", "+calculatedLeftY+", "+calculatedRightY);
    drivetrain.drive(calculatedLeftX, calculatedLeftY, calculatedRightY, getPeriod());
  }

}