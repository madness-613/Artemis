// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
private final XboxController controler = new XboxController(0);
private double revToWheel = 198/2108;
private double revToFoot = 1188*Math.PI/25296;
private double errorboost = 0.5;
private double setPoint = 3;
private double setangle = 0;

private CANSparkMax frontLeft = new CANSparkMax(4, MotorType.kBrushless);
private CANSparkMax frontRight = new CANSparkMax(3, MotorType.kBrushless);
private CANSparkMax backLeft = new CANSparkMax(2, MotorType.kBrushless);
private CANSparkMax backRight = new CANSparkMax(1, MotorType.kBrushless);

private MotorControllerGroup left = new MotorControllerGroup(frontLeft, backLeft);
private MotorControllerGroup Right = new MotorControllerGroup(frontRight, backRight);

private RelativeEncoder frontLeftEncoder = frontLeft.getEncoder(); 
private RelativeEncoder frontRightEncoder = frontRight.getEncoder();
private RelativeEncoder backLeftEncoder = backLeft.getEncoder(); 
private RelativeEncoder backRightEncoder = backRight.getEncoder();

private WPI_Pigeon2 gyro = new WPI_Pigeon2(6);

NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
NetworkTableEntry tx = table.getEntry("tx");
NetworkTableEntry ty = table.getEntry("ty");
NetworkTableEntry ta = table.getEntry("ta");

private DifferentialDrive drive = new DifferentialDrive(left, Right);
  @Override
  public void robotInit() {
    drive.setMaxOutput(0.2);
    left.setInverted(true);
    frontLeftEncoder.setPosition(0);
    frontRightEncoder.setPosition(0);
    backLeftEncoder.setPosition(0);
    backRightEncoder.setPosition(0);
    frontLeftEncoder.setPositionConversionFactor(revToFoot);
    frontRightEncoder.setPositionConversionFactor(revToFoot);
    backLeftEncoder.setPositionConversionFactor(revToFoot);
    backRightEncoder.setPositionConversionFactor(revToFoot);
  } 
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("front left encoder", frontLeftEncoder.getPosition());
    SmartDashboard.putNumber("front right encoder", frontRightEncoder.getPosition());
    SmartDashboard.putNumber("back left encoder", backLeftEncoder.getPosition());
    SmartDashboard.putNumber("back right encoder", backRightEncoder.getPosition());
    SmartDashboard.putNumber("gyro angle", gyro.getAngle());
    SmartDashboard.putNumber("gyro pitch", gyro.getPitch());
    SmartDashboard.putNumber("gyro roll", gyro.getRoll()); 
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
  }
  @Override
  public void autonomousInit() {

  }
  @Override
  public void autonomousPeriodic() {
    
  }
  @Override
  public void teleopInit() {
    gyro.setYaw(0);
  }
  @Override
  public void teleopPeriodic() {
    double gyroError = setangle - gyro.getAngle();
    SmartDashboard.putNumber("gyro error", gyroError); 
    double speed = controler.getLeftY();
    double turn = controler.getRightX();
    drive.tankDrive(speed + gyroError * errorboost, turn - gyroError * errorboost);
  }
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
