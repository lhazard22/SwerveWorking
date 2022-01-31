// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.fasterxml.jackson.databind.ser.impl.FailingSerializer;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.opencv.calib3d.StereoBM;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.SerialPort;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  XboxController driverController = new XboxController(0);
  // AHRS gyro = new AHRS(SerialPort.Port.kMXP);
  CANSparkMax flDrive = new CANSparkMax(3, MotorType.kBrushless);
  CANSparkMax flSteer = new CANSparkMax(4, MotorType.kBrushless);
  CANSparkMax frDrive = new CANSparkMax(6, MotorType.kBrushless);
  CANSparkMax frSteer = new CANSparkMax(5, MotorType.kBrushless);
  CANSparkMax blDrive = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkMax blSteer = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax brDrive = new CANSparkMax(7, MotorType.kBrushless);
  CANSparkMax brSteer = new CANSparkMax(8, MotorType.kBrushless);
  DutyCycleEncoder blEncoder = new DutyCycleEncoder(0);
  DutyCycleEncoder flEncoder = new DutyCycleEncoder(1);
  DutyCycleEncoder frEncoder = new DutyCycleEncoder(2);
  DutyCycleEncoder brEncoder = new DutyCycleEncoder(3);
  SpeedControllerGroup driveMotors = new SpeedControllerGroup(flDrive, frDrive, blDrive, brDrive);
  SpeedControllerGroup steerMotors = new SpeedControllerGroup(flSteer, frSteer, blSteer, brSteer);
  double deadzone = 0.1;
  double zeroTolerance = 0.3;
  double steerGearReduction = 1000 / 8.16;
  double driveGearReduction = 1000 / 12.8;
  double x = 0;
  double rot = 0;
  double y = 0;
  double w1ca = 0;
  double w2ca = 0;
  double w3ca = 0;
  double w4ca = 0;
  double w1Angle;

  double counter = 0;

  double length = 0.47; // meters module center point to module center point
  double width = 0.47; // meters module center point to module center point
  double chassisRadius = Math.sqrt(Math.pow(length, 2) + Math.pow(width, 2)) / 2;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    flDrive.setInverted(false);
    flSteer.setInverted(true);
    frDrive.setInverted(false);
    frSteer.setInverted(true);
    blDrive.setInverted(false);
    blSteer.setInverted(true);
    brDrive.setInverted(false);
    brSteer.setInverted(true);
    brSteer.setIdleMode(CANSparkMax.IdleMode.kBrake);
    blSteer.setIdleMode(CANSparkMax.IdleMode.kBrake);
    frSteer.setIdleMode(CANSparkMax.IdleMode.kBrake);
    flSteer.setIdleMode(CANSparkMax.IdleMode.kBrake);
    counter = 0;

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

  }

  /** This function is called once each time the robot enters Disabled mode. */
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

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // TODO get angle fron gyro

    //Finds the X Value of the Left Stick on the Controller and Takes Care of Joystick Drift
    if (Math.abs(driverController.getX(Hand.kLeft)) < deadzone) {
      x = 0;
    } else {
      x = driverController.getX(Hand.kLeft);
    }

    //Finds the Y Value of the Left Stick on the Controller and Takes Care of Joystick Drift
    if (Math.abs(driverController.getY(Hand.kLeft)) < deadzone) {
      y = 0;
    } else {
      y = -driverController.getY(Hand.kLeft);
    }

    //Finds the X Value of the Right Stick on the Controller and Takes Care of Joystick Drift
    if (Math.abs(driverController.getX(Hand.kRight)) < deadzone) {
      rot = 0;
    } else {
      rot = driverController.getX(Hand.kRight);
    }

    double vx = x;// desired x speed
    double vy = y;// desired y speed
    double omega = rot;// desired rotation, clockwise pos in deg Check?

    double A = vx - omega * length / 2;
    double B = vx + omega * length / 2;
    double C = vy - omega * length / 2;
    double D = vy + omega * length / 2;

    
    //Finds Speeds for Each of the Wheels
    double w1s = Math.sqrt(Math.pow(B, 2) + Math.pow(C, 2)) / 10;
    double w2s = Math.sqrt(Math.pow(B, 2) + Math.pow(D, 2)) / 10;
    double w3s = Math.sqrt(Math.pow(A, 2) + Math.pow(D, 2)) / 10;
    double w4s = Math.sqrt(Math.pow(A, 2) + Math.pow(C, 2)) / 10;


    //Finds the Desired Angle
    double w1a = (Math.atan2(B, C) * (180 / Math.PI)) + 180;
    double w2a = (Math.atan2(B, D) * (180 / Math.PI)) + 180;
    double w3a = (Math.atan2(A, D) * (180 / Math.PI)) + 180;
    double w4a = (Math.atan2(A, C) * (180 / Math.PI)) + 180;

    
    //Manipulates Degree Values so 0 is on top and degree values get bigger when going clockwise
    if (w1a == 360) {
      w1a = 0;
    }
    if (w2a == 360) {
      w2a = 0;
    }
    if (w3a == 360) {
      w3a = 0;
    }
    if (w4a == 360) {
      w4a = 0;
    }

    if (w1a <= 180) {
      w1a = w1a + 180;
    } else if (w1a > 180) {
      w1a = w1a - 180;
    }

    if (w2a <= 180) {
      w2a = w2a + 180;
    } else if (w2a > 180) {
      w2a = w2a - 180;
    }
    
    if (w3a <= 180) {
      w3a = w3a + 180;
    } else if (w3a > 180) {
      w3a = w3a - 180;
    }

    if (w4a <= 180) {
      w4a = w4a + 180;
    } else if (w4a > 180) {
      w4a = w4a - 180;
    }


    if (w1a == 360) {
      w1a = 0;
    }
    if (w2a == 360) {
      w2a = 0;
    }
    if (w3a == 360) {
      w3a = 0;
    }
    if (w4a == 360) {
      w4a = 0;
    }
    
    //Finds Complimentary Angle to the Desired Angle
    double w1ra = getRevPosition(w1a);
    double w2ra = getRevPosition(w2a);
    double w3ra = getRevPosition(w3a);
    double w4ra = getRevPosition(w4a);

    //Sets Max Wheel Speed
    double max = w1s;
    if (w2s > max)
      max = w2s;
    if (w3s > max)
      max = w3s;
    if (w4s > max)
      max = w4s;
    if (max > 1) {
      w1s = w1s / max;
      w2s = w2s / max;
      w3s = w3s / max;
      w4s = w4s / max;
    }
    
    //Sets the Tolerance of Steer Motors to being off of desired angle 
    double spinTolerance = 5;

    //Finds Actual Angle of Wheels
    w1ca = (-1 * getPosition(frEncoder.get(), 267.4)) + 360;
    w2ca = (-1 * getPosition(flEncoder.get(), 120.7)) + 360;
    w3ca = (-1 * getPosition(blEncoder.get(), 64.7)) + 360;
    w4ca = (-1 * getPosition(brEncoder.get(), 335.2)) + 360;

    if (w1ca == 360) {
      w1a = 0;
    }
    if (w2ca == 360) {
      w2a = 0;
    }
    if (w3ca == 360) {
      w3a = 0;
    }
    if (w4ca == 360) {
      w4a = 0;
    }

    double w1d_1 = w1a - w1ca;
    double w1d_2 = w1ca - w1a;
    double w1d_3 = w1ra - w1ca;
    double w1d_4 = w1ca - w1ra;

    double w2d_1 = w2a - w2ca;
    double w2d_2 = w2ca - w2a;
    double w2d_3 = w2ra - w2ca;
    double w2d_4 = w2ca - w2ra;

    double w3d_1 = w3a - w3ca;
    double w3d_2 = w3ca - w3a;
    double w3d_3 = w3ra - w3ca;
    double w3d_4 = w3ca - w3ra;

    double w4d_1 = w4a - w4ca;
    double w4d_2 = w4ca - w4a;
    double w4d_3 = w4ra - w4ca;
    double w4d_4 = w4ca - w4ra;


    if (w1d_1 < 0) {
      w1d_1 = w1d_1 + 360;
    }
    if (w1d_2 < 0) {
      w1d_2 = w1d_2 + 360;
    }
    if (w1d_3 < 0) {
      w1d_3 = w1d_3 + 360;
    }
    if (w1d_4 < 0) {
      w1d_4 = w1d_4 + 360;
    }

    if (w2d_1 < 0) {
      w2d_1 = w2d_1 + 360;
    }
    if (w2d_2 < 0) {
      w2d_2 = w2d_2 + 360;
    }
    if (w2d_3 < 0) {
      w2d_3 = w2d_3 + 360;
    }
    if (w2d_4 < 0) {
      w2d_4 = w2d_4 + 360;
    }

    if (w3d_1 < 0) {
      w3d_1 = w3d_1 + 360;
    }
    if (w3d_2 < 0) {
      w3d_2 = w3d_2 + 360;
    }
    if (w3d_3 < 0) {
      w3d_3 = w3d_3 + 360;
    }
    if (w3d_4 < 0) {
      w3d_4 = w3d_4 + 360;
    }

    if (w4d_1 < 0) {
      w4d_1 = w4d_1 + 360;
    }
    if (w4d_2 < 0) {
      w4d_2 = w4d_2 + 360;
    }
    if (w4d_3 < 0) {
      w4d_3 = w4d_3 + 360;
    }
    if (w4d_4 < 0) {
      w4d_4 = w4d_4 + 360;
    }


    double value_1 = getOptimalRoute(w1d_1, w1d_2, w1d_3, w1d_4);
    double value_2 = getOptimalRoute(w2d_1, w2d_2, w2d_3, w2d_4);
    double value_3 = getOptimalRoute(w3d_1, w3d_2, w3d_3, w3d_4);
    double value_4 = getOptimalRoute(w4d_1, w4d_2, w4d_3, w4d_4);
    

    if (value_1 == 1) {
      frDrive.set(w1s);
      if (w1d_1 > spinTolerance) {
        frSteer.set(0.07);
      } else {
        frSteer.set(0);
      }
    } else if (value_1 == 2) {
      frDrive.set(w1s);
      if (w1d_2 > spinTolerance) {
        frSteer.set(-0.07);
      } else {
        frSteer.set(0);
      }
    } else if (value_1 == 3) {
      frDrive.set(-w1s);
      if (w1d_3 > spinTolerance) {
        frSteer.set(0.07);
      } else {
        frSteer.set(0);
      }
    } else if (value_1 == 4) {
      frDrive.set(-w1s);
      if (w1d_4 > spinTolerance) {
        frSteer.set(-0.07);
      } else {
        frSteer.set(0);
      }
    }

    if (value_2 == 1) {
      flDrive.set(w2s);
      if (w2d_1 > spinTolerance) {
        flSteer.set(0.07);
      } else {
        flSteer.set(0);
      }
    } else if (value_2 == 2) {
      flDrive.set(w2s);
      if (w2d_2 > spinTolerance) {
        flSteer.set(-0.07);
      } else {
        flSteer.set(0);
      }
    } else if (value_2 == 3) {
      flDrive.set(-w2s);
      if (w2d_3 > spinTolerance) {
        flSteer.set(0.07);
      } else {
        flSteer.set(0);
      }
    } else if (value_2 == 4) {
      flDrive.set(-w2s);
      if (w2d_4 > spinTolerance) {
        flSteer.set(-0.07);
      } else {
        flSteer.set(0);
      }
    }

    if (value_3 == 1) {
      blDrive.set(w3s);
      if (w3d_1 > spinTolerance) {
        blSteer.set(0.07);
      } else {
        blSteer.set(0);
      }
    } else if (value_3 == 2) {
      blDrive.set(w3s);
      if (w3d_2 > spinTolerance) {
        blSteer.set(-0.07);
      } else {
        blSteer.set(0);
      }
    } else if (value_3 == 3) {
      blDrive.set(-w3s);
      if (w3d_3 > spinTolerance) {
        blSteer.set(0.07);
      } else {
        blSteer.set(0);
      }
    } else if (value_3 == 4) {
      blDrive.set(-w3s);
      if (w3d_4 > spinTolerance) {
        blSteer.set(-0.07);
      } else {
        blSteer.set(0);
      }
    }

    if (value_4 == 1) {
      brDrive.set(w4s);
      if (w4d_1 > spinTolerance) {
        brSteer.set(0.07);
      } else {
        brSteer.set(0);
      }
    } else if (value_4 == 2) {
      brDrive.set(w4s);
      if (w4d_2 > spinTolerance) {
        brSteer.set(-0.07);
      } else {
        brSteer.set(0);
      }
    } else if (value_4 == 3) {
      brDrive.set(-w4s);
      if (w4d_3 > spinTolerance) {
        brSteer.set(0.07);
      } else {
        brSteer.set(0);
      }
    } else if (value_4 == 4) {
      brDrive.set(-w4s);
      if (w4d_4 > spinTolerance) {
        brSteer.set(-0.07);
      } else {
        brSteer.set(0);
      }
    }

    
    // double w1d = w1ca - w1a;
    // double w2d = w2ca - w2a;
    // double w3d = w3ca - w3a;
    // double w4d = w4ca - w4a;

    // System.out.println(getPosition(frEncoder.get(), 267.4) + "Encoder FL");
    // System.out.println(getPosition(flEncoder.get(), 120.7) + "Encoder FL");
    // System.out.println(getPosition(brEncoder.get(), 335.2) + "Encoder BR");
    // System.out.println(getPosition(blEncoder.get(), 64.7) + "Encoder BL");
    /*
     * if (counter == 10) { System.out.format("Desired " + "%.1f%n", w1a);
     * System.out.format("Desired Aswell " + "%.1f%n", w1Angle);
     * System.out.format("Current " + "%.1f%n", w1ca);
     * System.out.format("Difference " + "%.1f%n%n", w1d); System.out.println(" ");
     * counter = 0; } counter = counter + 1;
     */
    // frDrive.set(w1s);
    /*
     * if (w1d > spinTolerance) { if (Math.abs(w1a - w1ca) < 180) {
     * frSteer.set(-0.07); } else { frSteer.set(0.07); }
     * 
     * } else { frSteer.set(0); }
     */

    /*
     * if (w1ca > 90 && w1a < -90) { if (((Math.abs(w1ca - 360)) - w1a) < 90) {
     * frSteer.set(-0.07); } else { frSteer.set(0.07); } } else if (w1ca < -90 &&
     * w1a > 90) { if (((Math.abs(w1ca + 360)) - w1a) < 90) { frSteer.set(0.07); }
     * else { frSteer.set(-0.07); } } else { if (w1d < 0) { frSteer.set(-0.07); }
     * else { frSteer.set(0.07); } }
     */

    /*
     * if (Math.abs(w1d) < spinTolerance) { frSteer.set(0); } else { if
     * (Math.abs(w1d) > 90) { if (w1a >= 0) { w1a = w1a - 180; } else { w1a = w1a +
     * 180; } } if (w1ca > 90 && w1a < -90) { if (((Math.abs(w1ca - 360)) - w1a) <
     * 90) { frSteer.set(-0.07); } else { frSteer.set(0.07); } } else if (w1ca < -90
     * && w1a > 90) { if (((Math.abs(w1ca + 360)) - w1a) < 90) { frSteer.set(0.07);
     * } else { frSteer.set(-0.07); } } else { if (w1d < 0) { frSteer.set(-0.07); }
     * else { frSteer.set(0.07); } } }
     * 
     * if (Math.abs(w2d) < spinTolerance) { flSteer.set(0); } else { if
     * (Math.abs(w2d) > 90) { if (w2a >= 0) { w2a = w2a - 180; } else { w2a = w2a +
     * 180; } } if (w2ca > 90 && w2a < -90) { if (((Math.abs(w2ca - 360)) - w2a) <
     * 90) { flSteer.set(-0.07); } else { flSteer.set(0.07); } } else if (w2ca < -90
     * && w2a > 90) { if (((Math.abs(w2ca + 360)) - w2a) < 90) { flSteer.set(0.07);
     * } else { flSteer.set(-0.07); } } else { if (w2d < 0) { flSteer.set(-0.07); }
     * else { flSteer.set(0.07); } } }
     * 
     * if (Math.abs(w3d) < spinTolerance) { blSteer.set(0); } else { if
     * (Math.abs(w3d) > 90) { if (w3a >= 0) { w3a = w3a - 180; } else { w3a = w3a +
     * 180; } }
     * 
     * if (w3ca > 90 && w3a < -90) { if (((Math.abs(w3ca - 360)) - w3a) < 90) {
     * blSteer.set(-0.07); } else { blSteer.set(0.07); } } else if (w3ca < -90 &&
     * w3a > 90) { if (((Math.abs(w3ca + 360)) - w3a) < 90) { blSteer.set(0.07); }
     * else { blSteer.set(-0.07); } } else { if (w3d < 0) { blSteer.set(-0.07); }
     * else { blSteer.set(0.07); } } }
     * 
     * if (Math.abs(w4d) < spinTolerance) { brSteer.set(0); } else { if
     * (Math.abs(w4d) > 90) { if (counter == 0) { System.out.println(w4a); } if (w4a
     * >= 0) { w4a = w4a - 180; System.out.println("B"); } else { w4a = w4a + 180;
     * if (counter == 0) { System.out.println(w4a); System.out.println("A"); counter
     * = counter + 1; } } } if (w4ca > 90 && w4a < -90) { if (((Math.abs(w4ca -
     * 360)) - w4a) < 90) { brSteer.set(-0.07); } else { brSteer.set(0.07); } } else
     * if (w4ca < -90 && w4a > 90) { if (((Math.abs(w4ca + 360)) - w4a) < 90) {
     * brSteer.set(0.07); } else { brSteer.set(-0.07); } } else { if (w4d < 0) {
     * brSteer.set(-0.07); } else { brSteer.set(0.07); } } }
     * 
     * frDrive.set(w1s / 3);
     * 
     * flDrive.set(w2s / 3);
     * 
     * blDrive.set(w3s / 3);
     * 
     * brDrive.set(w4s / 3);
     */

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.

  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  public double getPosition(double rawAngle, double offset) {
    double offsetRot = offset / 360;
    double angle = rawAngle - offsetRot;
    double angleDeg = (angle % 1) * 360;
    if (angleDeg < 0) {
      angleDeg = angleDeg + 360;
    }

    return angleDeg;
  }

  public double getRawPosition(double rawAngle, double offset) {
    double offsetRot = offset / 360;
    double angle = rawAngle - offsetRot;
    double angleRaw = (angle % 1);
    if (angleRaw < 0) {
      angleRaw = angleRaw + 1;
    }
    return angleRaw;
  }

  public double getRevPosition(double angle) {
    if (angle >= 180) {
      return angle - 180;
    } else {
      return angle + 180;
    }

  }

  public double setDifferenceRollover(double dif) {
    if (dif < 0) {
      return dif = dif + 360;
    } else {
      return dif;
    }
  }

  public double getOptimalRoute(double dif1, double dif2, double dif3, double dif4) {
    if (dif1 <= dif2 && dif1 <= dif3 && dif1 <= dif4) {
      return 1;
    } else if (dif2 <= dif1 && dif2 <= dif3 && dif2 <= dif4) {
      return 2;
    } else if (dif3 <= dif1 && dif3 <= dif2 && dif3 <= dif4) {
      return 3;
    } else if (dif4 <= dif1 && dif4 <= dif2 && dif4 <= dif3) {
      return 4;
    } else {
      return 0;
    }
  }
  // Offsets
  // FR = 267.4
  // FL = 120.7
  // BR = 335.2
  // BL = 64.7

}
