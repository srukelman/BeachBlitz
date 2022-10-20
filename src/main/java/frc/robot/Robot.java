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

import javax.swing.RowFilter.ComparisonType;

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
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    //System.out.println("Auto selected: " + m_autoSelected);
  }
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
  @Override
  public void teleopInit() {
    
  }
  @Override
  public void teleopPeriodic() {
    //Stick trigger --> shift drive
    //xbox left bumper --> extend intake
    //xbox right bumper --> run hopper
    //xbox B button --> drive intake
    //right trigger --> drive neos
    updatePID();
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
    if(xbox.getBButtonPressed()){
      ty = table.getEntry("ty");
      targetOffsetAngle_Vertical = ty.getDouble(10.0);

      // how many degrees back is your limelight rotated from perfectly vertical?
      limelightMountAngleDegrees = 45.0;

      // distance from the center of the Limelight lens to the floor
      limelightLensHeightInches = 7.5;

      // distance from the target to the floor
      goalHeightInches = 92.0;

      angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
      angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);
      System.out.println(angleToGoalDegrees);
      distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)/Math.tan(angleToGoalRadians);
      System.out.println(distanceFromLimelightToGoalInches);
    }
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
      SmartDashboard.putNumber("Actual RPM", rpm);
      //double linearSpeed = (rpm*4*3.1415/12)/60;
      double feetPerSecond = SmartDashboard.getNumber("FLYWHEEL TARGET", 0);
      double setPoint = (FPSTORPM(feetPerSecond)<maxRPM)?FPSTORPM(feetPerSecond):maxRPM;
      leftPID.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
      rightPID.setReference(-setPoint, CANSparkMax.ControlType.kVelocity);
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
  public double[] getTargetRPM(double distance){
    return new double[]{0,0,0,0};
  }
  public double FPSTORPM(double fps){
    return ((fps * 60 * 12)/3.14159)/4;
  }
  public double hoodReadingtoDegrees(double hoodReading){
    return 5d+((hoodMotor.getSelectedSensorPosition()/7672d)*40d);
  }
  public void updatePID(){
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { leftPID.setP(p); rightPID.setP(p); kP = p; }
    if((i != kI)) { leftPID.setI(i); rightPID.setI(i); kI = i; }
    if((d != kD)) {leftPID.setD(d); rightPID.setD(d); kD = d; }
    if((iz != kIz)) { leftPID.setIZone(iz); rightPID.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { leftPID.setFF(ff); rightPID.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      leftPID.setOutputRange(min, max); 
      rightPID.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max;
    }
  }
}
