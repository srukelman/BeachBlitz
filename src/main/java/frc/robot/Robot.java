package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Encoder;

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
  CANSparkMax rightNeo = new CANSparkMax(2, MotorType.kBrushless);
  SparkMaxPIDController leftPID = leftNeo.getPIDController();
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
PIDController hPidController = new PIDController(0.0008, 0, 0);

double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);
double leftMax = 0;
double rightMax = 0;
double feetPerSecond = 0;
double hoodTarget = 5;
double targetRPM = 0;
ShuffleboardTab tab = Shuffleboard.getTab("BB");
Joystick js = new Joystick(0);
double setPoint = 9000;

//calculate distance
double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)/Math.tan(angleToGoalRadians);

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
    leftNeo.setIdleMode(IdleMode.kCoast);
    rightNeo.setIdleMode(IdleMode.kCoast);
    rightNeo.follow(leftNeo, true);
    kP = 6e-5;
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0.000015;
    kMaxOutput = .85;
    kMinOutput = -.85;
    maxRPM = 5700;
    leftPID.setP(kP);
    leftPID.setI(kI);
    leftPID.setD(kD);
    leftPID.setIZone(kIz);
    leftPID.setFF(kFF);
    leftPID.setOutputRange(kMinOutput, kMaxOutput);
    leftNeo.burnFlash();
    rightNeo.burnFlash();
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putBoolean("drivetrain solenoid", dogShifter.get());
    SmartDashboard.putNumber("HOOD TARGET", hoodTarget);
    SmartDashboard.putNumber("hood angle", hoodMotor.getSelectedSensorPosition());
    SmartDashboard.putBoolean("Target", tv.getDouble(0)==1);
    SmartDashboard.putNumber("FLYWHEEL TARGET", targetRPM);
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putBoolean("Conpressor On", c.enabled());
  }
  @Override
  public void autonomousInit() {
  }
  @Override
  public void autonomousPeriodic() {
  }
  @Override
  public void teleopInit() {
    
  }
  @Override
  public void teleopPeriodic() {
    if(xbox.getLeftBumperPressed()){
      intakeSolenoid.toggle();
    }
    if(js.getTriggerPressed()){
      if(hoodTarget!= 50){
        hoodTarget +=5;
      }    
    }
    if(js.getRawButtonPressed(2)){
      if(hoodTarget!=5){
       hoodTarget -=5;
      }
    }
    if(js.getRawButtonPressed(3)){
      targetRPM+=100;
    }
    if(js.getRawButtonPressed(4)){
      targetRPM-=100;
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
        speed = xbox.getRawAxis(1) * xbox.getRawAxis(1) * .75;
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
    // if(xbox.getBackButton()){
    //     PIDController hPidController = new PIDController(0.0008, 0, 0);
    //     hoodMotor.set(ControlMode.PercentOutput, hPidController.calculate(hoodMotor.getSelectedSensorPosition(), degreesToHoodReading(hoodTarget)));
    // }else{
    //   hoodMotor.set(ControlMode.PercentOutput, 0);
    // }
    if(xbox.getRawAxis(3)> 0){
      double rpm = leftNeo.getEncoder().getVelocity();
      SmartDashboard.putNumber("Actual RPM", rpm);
      feetPerSecond = SmartDashboard.getNumber("FLYWHEEL TARGET", 0);
      if(tv.getDouble(0)==1){
        double horizontalOffset = tx.getDouble(0);
        if(horizontalOffset >=.05 || horizontalOffset <= -.05){
          LeftMaster.set(ControlMode.PercentOutput, .75*limeController.calculate(horizontalOffset, 0));
          RightMaster.set(ControlMode.PercentOutput, .75*limeController.calculate(horizontalOffset, 0));
          setPoint = 9000;
          hoodMotor.set(ControlMode.PercentOutput, 0);
          leftNeo.set(0);
        }else{
          LeftMaster.set(ControlMode.PercentOutput, 0);
          RightMaster.set(ControlMode.PercentOutput,0);
          ty = table.getEntry("ty");
          double verticalOffset = ty.getDouble(0.0);
          double angle = (verticalOffset+40)*(3.14159/180);
          double distanceFromLimelightToGoalInches = (104-29.5)/Math.tan(angle);
          SmartDashboard.putNumber("distance", distanceFromLimelightToGoalInches);
          hoodTarget = distanceToHoodAngle(distanceFromLimelightToGoalInches);
          targetRPM = distanceToRPM(distanceFromLimelightToGoalInches);
          if(rpm < targetRPM-25){
            setPoint += 100;
          }else if(rpm > targetRPM+25){
            setPoint -= 100;
          }
          else if(degreesToHoodReading(hoodTarget)-15<=hoodMotor.getSelectedSensorPosition() && degreesToHoodReading(hoodTarget)+15>=hoodMotor.getSelectedSensorPosition()){
            feed.set(ControlMode.PercentOutput, .5);
          }
          SmartDashboard.putNumber("setpoint", setPoint);
          leftPID.setReference(setPoint, ControlType.kVelocity);
          if(degreesToHoodReading(hoodTarget)-15>hoodMotor.getSelectedSensorPosition() || degreesToHoodReading(hoodTarget)+15<hoodMotor.getSelectedSensorPosition()){
            hoodMotor.set(ControlMode.PercentOutput, hPidController.calculate(hoodMotor.getSelectedSensorPosition(), degreesToHoodReading(hoodTarget)));
          }else{
            hoodMotor.set(ControlMode.PercentOutput, 0);
          }
        }
      }
    }
    else{
      leftNeo.set(0);
      feed.set(ControlMode.PercentOutput, 0);
      hoodMotor.set(ControlMode.PercentOutput, 0);
    }
    // if(xbox.getAButton()){
    //   feed.set(ControlMode.PercentOutput, .5);
    // }else{
    //   feed.set(ControlMode.PercentOutput, 0);  
    // }    
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
    return 5d+((hoodReading/7672d)*45d);
  }
  public double degreesToHoodReading(double degrees){
    if(degrees<5){
      return 0;
    }
    return (degrees-5d)*7672d/45d;
  }
  public double distanceToRPM(double distance){
    return 1000*(.296*Math.tan((distance - 10)/6)+3.45);
  }
  public double distanceToHoodAngle(double distance){
    return (20/(1+ Math.pow(Math.E, (-.793*(distance - 10)))))+25;
  }
}
