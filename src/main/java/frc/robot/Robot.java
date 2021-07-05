// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  private final TalonSRX tal_master = new TalonSRX(0); // declare and initialize Talon motor
  private final TalonSRX tal_follow = new TalonSRX(1); // declare and initialize Talon motor
  private final Joystick m_stick = new Joystick(0);// declare and initialize Joystick

  // declaration for absolute encoder
  final int kTimeoutMs = 30;
  final int kBookEnd_0 = 910; // start from 80 degree (80 / 360 * 4096)
  final int kBookEnd_1 = 1137; // end at 100 degree (100 / 360 * 4096)

  int mode;
  // 0 for relative encoder
  // 1 for absolute encoder
  // 2 for motion magic
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    tal_master.configFactoryDefault(); // reset to default
    tal_follow.configFactoryDefault(); // reset to default

    tal_follow.follow(tal_master);
    // one follow anothwe to ensure two rotate almost exactly the same
    tal_follow.setInverted(false);
    tal_master.setInverted(false);
    // if you find that when you give a positive value and the motor doesn't rotate along the distance you expect
    // you can set it to rotate inversingly instead of giving it a negative value
    tal_master.setSensorPhase(false);
    // to set sensor output the same direction as the motor output

    if(mode == 0) {
      tal_master.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
      // set feedback sensor as relative encoder
      // encoder is wired directly onto the motor controller so no more initialization
      tal_master.setSelectedSensorPosition(0);
      // similar to gyro's "reset to zero"
    }
    else if(mode == 1) {
      tal_master.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, kTimeoutMs);
      // the second parameter: 0 for primary closed loop and 1 for auxiliary closed loop
      // set Timeout time as non zero to check if the configuration success before the time you set
      int pulseWidth = tal_master.getSensorCollection().getPulseWidthPosition();
      int newCenter = (kBookEnd_0 + kBookEnd_1)/2;
      newCenter &= 0xFFF; // and the number with hex of 4095 to make sure it doesn't exceed 4096
      pulseWidth -= newCenter;
      pulseWidth &= 0xFFF;
      tal_master.getSensorCollection().setQuadraturePosition(pulseWidth, kTimeoutMs);
      // to change the quadrature "reported" position
      // calculate the difference between your position and the center you set
      // so when set the center as zero, the diffence will be the value of your current position
    }
    else if(mode == 2) {
      tal_master.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
      tal_master.configNeutralDeadband(0.001);
      // to guess(?) its deadband, the function will make sure the current value it generates is higher than it
      // if you set too low, lower than the actual deadband, the robot might not move
      // if you set too high, higher than the actual deadband, it might be too high so the robot might damp
      tal_master.selectProfileSlot(0, 0);
      // the first parameter: Talon's ID
      // the second parameter: 0 for primary closed loop and 1 for auxiliary closed loop
      tal_master.config_kP(0, 0.2); // talon id, p value
      tal_master.config_kI(0, 0); // talon id, p value
      tal_master.config_kD(0, 0); // talon id, p value
      tal_master.config_kF(0, 0.2); // talon id, p value

      tal_master.configMotionCruiseVelocity(800); //sensorUnitsPer100ms
      tal_master.configMotionAcceleration(200); //sensorUnitsPer100msPerSec
      // set cruise velocity(maximum), acceleration, acceleration smoothing
      // if you don't set them it will use default values
      tal_master.setSelectedSensorPosition(0);
      // similar to gyro's "reset to zero"
    }
    
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // for relative encoder
    // we move the robot parts teleoperatedly and check its position and velocity instantly
    if(Math.abs(m_stick.getRawAxis(5)) > 0.1)
      tal_master.set(ControlMode.PercentOutput, 0.3 * m_stick.getRawAxis(5));
    else
      tal_master.set(ControlMode.PercentOutput, 0);
    // check axis's absolute value > 0.1 to avoid wrong actions for touching axis accidentally
    // Different control mode
    // Percent output is the standard mode that gives the motor a value between -1 to 1

    double vel = tal_master.getSelectedSensorVelocity();
    double pos = tal_master.getSelectedSensorPosition();
    SmartDashboard.putNumber("velocity", vel);
    SmartDashboard.putNumber("position", pos);
    // functions to get velocity and position
    // use smart dashboard (actually shuffle board) to show the result
  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    double leftYstick = 0;
    if(Math.abs(m_stick.getY()) > 0.1)
      leftYstick = m_stick.getY();
    if(m_stick.getRawButton(2)) {
      double targetPosition = leftYstick * 4096.0;
      tal_master.set(ControlMode.MotionMagic, targetPosition);
    }
    // after you set all your initialization during init
    // you just give it your target and it will design the perfect way to reach it for you
    double vel = tal_master.getSelectedSensorVelocity();
    double pos = tal_master.getSelectedSensorPosition();
    SmartDashboard.putNumber("velocity", vel);
    SmartDashboard.putNumber("position", pos);
    // functions to get velocity and position
    // use smart dashboard (actually shuffle board) to show the result
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  public double ToDeg(double units) {
    double deg = units * 360.0 / 4096.0;
    deg *= 10;
    deg = (int)deg;
    deg /= 10;
    // 2003.3 -> 20033 -> 2003
    // when you divide a int you will onlt get a int
    return deg;
  }
  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    // this part is for absolute encoder
    if(m_stick.getRawButton(1)) {
      // for the convenience of initialization
      int pulseWidth = tal_master.getSensorCollection().getPulseWidthPosition();
      int newCenter = (kBookEnd_0 + kBookEnd_1)/2;
      newCenter &= 0xFFF; // and the number with hex of 4095 to make sure it doesn't exceed 4096
      pulseWidth -= newCenter;
      pulseWidth &= 0xFFF;
      tal_master.getSensorCollection().setQuadraturePosition(pulseWidth, kTimeoutMs);
    }
    if(Math.abs(m_stick.getRawAxis(1)) > 0.1) 
      tal_master.set(ControlMode.PercentOutput, 0.3 * m_stick.getRawAxis(5));
    else
      tal_master.set(ControlMode.PercentOutput, 0);
    if(m_stick.getRawButton(2)) {
      double selSenPos = tal_master.getSelectedSensorPosition();
      double pulseWidthWithoutOverflows = 
      tal_master.getSensorCollection().getPulseWidthPosition() & 0xFFF;

      double pulseWidthDeg = ToDeg(pulseWidthWithoutOverflows);
      double selSenDeg = ToDeg(selSenPos);

      SmartDashboard.putNumber("pulseWidthPos", pulseWidthWithoutOverflows);
      SmartDashboard.putNumber("selSenPos", selSenPos);
      SmartDashboard.putNumber("pulseWidthDeg", pulseWidthDeg);
      SmartDashboard.putNumber("selSenDeg", selSenDeg);
    }
      
  }
}
