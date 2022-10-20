// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  Compressor c;
  Solenoid dogShifter;
  Solenoid intakeSolenoid;
  TalonSRX LeftMaster = new TalonSRX(1);
  TalonSRX RightMaster = new TalonSRX(2);
  VictorSPX LeftSlave1 = new VictorSPX(5);
  VictorSPX LeftSlave2 = new VictorSPX(8);
  VictorSPX RightSlave1 = new VictorSPX(6);
  VictorSPX RightSlave2 = new VictorSPX(4);
  TalonSRX intakeMotor = new TalonSRX(3);
  TalonSRX lowerHopper = new TalonSRX(0);
  VictorSPX upperHopper = new VictorSPX(7);
  VictorSPX feed = new VictorSPX(2);
  CANSparkMax leftNeo = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax rightNeo = new CANSparkMax(0, MotorType.kBrushless);
  SparkMaxPIDController leftPID = leftNeo.getPIDController();
  SparkMaxPIDController rightPID = rightNeo.getPIDController();
  RelativeEncoder leftNeoEncoder = leftNeo.getEncoder();
  RelativeEncoder rightNeoEncoder = rightNeo.getEncoder();
  double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
  TalonSRX hoodMotor = new TalonSRX(9);
  double hoodPosition = hoodMotor.getSelectedSensorPosition();
  double hopperSpeed = 0.4;
  double intakeSpeed = 0.4;
  XboxController xbox;
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
NetworkTableEntry ty = table.getEntry("ty");
NetworkTableEntry tx = table.getEntry("tx");
NetworkTableEntry tv = table.getEntry("tv");
Encoder leftEncoder, rightEncoder;
double targetOffsetAngle_Vertical = ty.getDouble(0.0);

// how many degrees back is your limelight rotated from perfectly vertical?
double limelightMountAngleDegrees = 45.0;

// distance from the center of the Limelight lens to the floor
double limelightLensHeightInches = 39.0;

// distance from the target to the floor
double goalHeightInches = 92.0;
PIDController limeController = new PIDController(.04, .001, .00);

double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);
double leftMax = 0;
double rightMax = 0;
ShuffleboardTab tab = Shuffleboard.getTab("BB");

//calculate distance
double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)/Math.tan(angleToGoalRadians);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    c = new Compressor(1, PneumaticsModuleType.CTREPCM);
    xbox = new XboxController(1);
    rightEncoder = new Encoder(0,1 , true,EncodingType.k4X);
    leftEncoder = new Encoder(8,9, false, EncodingType.k4X);
    dogShifter = new Solenoid(1, PneumaticsModuleType.CTREPCM, 0);
    intakeSolenoid = new Solenoid(1, PneumaticsModuleType.CTREPCM, 1);
    LeftSlave1.follow(LeftMaster);
    LeftSlave2.follow(LeftMaster);
    RightSlave1.follow(RightMaster);
    RightSlave2.follow(RightMaster);
    LeftMaster.setInverted(false);
    upperHopper.setInverted(true);
    lowerHopper.setInverted(true);
    intakeMotor.setInverted(false);
    kP = 6e-5;
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0.000015;
    kMaxOutput = 1;
    kMinOutput = -1;
    maxRPM = 5700;
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putBoolean("drivetrain solenoid", dogShifter.get());
    SmartDashboard.putNumber("hood angle", hoodMotor.getSelectedSensorPosition());
    SmartDashboard.putBoolean("Target", tv.getDouble(0)==1);
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    //System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //Stick trigger --> shift drive
    //xbox left bumper --> extend intake
    //xbox right bumper --> run hopper
    //xbox B button --> drive intake
    //right trigger --> drive neos

    SmartDashboard.putBoolean("Conpressor On", c.enabled());
    if(c.enabled()){
      if(c.getPressureSwitchValue()){
        c.disable();
      }
    }
    if(xbox.getLeftBumperPressed()){
      intakeSolenoid.toggle();
    }
    if(xbox.getRightBumper()){
      lowerHopper.set(ControlMode.PercentOutput, hopperSpeed);
      upperHopper.set(ControlMode.PercentOutput, hopperSpeed);
    }
    else{
      lowerHopper.set(ControlMode.PercentOutput, 0);
      upperHopper.set(ControlMode.PercentOutput, 0);
    }
    if(xbox.getRawAxis(2)>0){
      intakeMotor.set(ControlMode.PercentOutput, intakeSpeed);
    }
    else{
      intakeMotor.set(ControlMode.PercentOutput, 0);
    }
    // if(xbox.getAButtonPressed()){
    //   ty = table.getEntry("ty");
    //   targetOffsetAngle_Vertical = ty.getDouble(10.0);

    //   // how many degrees back is your limelight rotated from perfectly vertical?
    //   limelightMountAngleDegrees = 45.0;

    //   // distance from the center of the Limelight lens to the floor
    //   limelightLensHeightInches = 7.5;

    //   // distance from the target to the floor
    //   goalHeightInches = 92.0;

    //   angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    //   angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);
    //   System.out.println(angleToGoalDegrees);
    //   distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)/Math.tan(angleToGoalRadians);
    //   System.out.println(distanceFromLimelightToGoalInches);
    // }
    if(xbox.getXButtonPressed()){
      c.disable();
    }
    if(xbox.getYButtonPressed()){
      c.enableAnalog(100, 120);
      c.enableDigital();
    }
    if(xbox.getStartButton()){
      ty = table.getEntry("ty");
      double verticalOffset = ty.getDouble(0.0);
      double angle = (verticalOffset+50)*(3.14159/180);
      double distanceFromLimelightToGoalInches = (104-29.5)/Math.tan(angle);
      SmartDashboard.putNumber("distance", distanceFromLimelightToGoalInches);
    }
    else{
      double speed, turn;
       if(xbox.getRawAxis(1) >= .1 || xbox.getRawAxis(1) <= -.1){
        speed = xbox.getRawAxis(1) * xbox.getRawAxis(1) * .85;
        speed = (xbox.getRawAxis(1) < 0)?(speed * -1):speed;
        turn = .15 * xbox.getRawAxis(4);
        LeftMaster.set(ControlMode.PercentOutput, -(speed+turn));
        RightMaster.set(ControlMode.PercentOutput, (speed-turn));
       }else if(xbox.getRawAxis(4) >= .1 || xbox.getRawAxis(4) <= -.1){
        turn = xbox.getRawAxis(4) * xbox.getRawAxis(4) * .65;
        turn = (xbox.getRawAxis(4) < 0)?(turn * -1):turn;
        LeftMaster.set(ControlMode.PercentOutput, turn);
        RightMaster.set(ControlMode.PercentOutput, turn);
       }
       else{
         LeftMaster.set(ControlMode.PercentOutput, 0);
         RightMaster.set(ControlMode.PercentOutput, 0);
       }
       if(leftEncoder.getRate() > leftMax){
        leftMax  = leftEncoder.getRate();
       }
       if(rightEncoder.getRate() > rightMax){
        rightMax = rightEncoder.getRate();
       }
    }
    if(xbox.getBackButton()){
        PIDController hPidController = new PIDController(0.0008, 0, 0);
        hoodMotor.set(ControlMode.PercentOutput, hPidController.calculate(hoodMotor.getSelectedSensorPosition(), SmartDashboard.getNumber("HOOD TARGET", 0)));
    }else{
      hoodMotor.set(ControlMode.PercentOutput, 0);
    }
    
    if(xbox.getRawAxis(3)> 0){
      double rpm = (leftNeo.getEncoder().getVelocity()-rightNeo.getEncoder().getVelocity())/2;
      double linearSpeed = (rpm*4*3.1415/12)/60;
      //PIDController neoPID = new PIDController(.05, 0, 0);
      // if(tv.getDouble(0)==1){
      //   ty = table.getEntry("ty");
      //   double verticalOffset = ty.getDouble(0.0);
      //   double angle = (verticalOffset+50)*(3.14159/180);
      //   distanceFromLimelightToGoalInches = (104-29.5)/Math.tan(angle);
      //   double[] targets = getTargetRPM(distanceFromLimelightToGoalInches);
      //   double targetRPM = targets[0];
      //   double targetAngle = tar
      // }
      double feetPerSecond = SmartDashboard.getNumber("FLYWHEEL TARGET", 0);
      
      leftNeo.set(1);
      rightNeo.set(-1);
    }
    else{
      leftNeo.set(0);
      rightNeo.set(0);
    }
    if(xbox.getAButton()){
      feed.set(ControlMode.PercentOutput, .5);
    }else{
      feed.set(ControlMode.PercentOutput, 0);
    }    
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  public double[] getTargetRPM(double distance){
    if(distance <= 5){
      return new double[]{FPSTORPM(22), 23.6, 7.6, 88d, 0};
    }
    if(distance <=10){
      return new double[]{FPSTORPM(25), 6.5, 8.2, 80d, 0};
    }
    if(distance <= 14){
      return new double[]{FPSTORPM(27), 5.6, 4.7, 70.3, 8.9};
    }
    if(distance <= 16){
      return new double[]{FPSTORPM(28), 40d, 15.3, 73d, 10.5};
    }
    if(distance <= 18){
      return new double[]{FPSTORPM(29), 28.3, 10.5, 58, 17.7};
    }
    if(distance <= 20){
      return new double[]{FPSTORPM(30), 28d, 9.4, 64.7, 17d};
    }
    if(distance <= 22){
      return new double[]{FPSTORPM(31), 20.8, 6.8, 61d, 20d};
    }
    if(distance <= 24){
      return new double[] {FPSTORPM(32), 58d, 29.6, 8.35, -3.5};
    }
    return new double[]{0,0,0,0};
  }
  public double FPSTORPM(double fps){
    return ((fps * 60 * 12)/3.14159)/4;
  }
  public double hoodReadingtoDegrees(double hoodReading){
    return 5d+((hoodMotor.getSelectedSensorPosition()/7672d)*40d);
  }
}
