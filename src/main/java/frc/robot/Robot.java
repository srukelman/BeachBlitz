// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
//import javax.swing.plaf.basic.BasicInternalFrameTitlePane.SystemMenuBar;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Counter;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.GenericHID.*;

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
  private VictorSPX LDriveV1 = new VictorSPX(6); //1st Left Drive Motor
  private VictorSPX LDriveV2 = new VictorSPX(7); //2nd Left Drive Motor
  private VictorSPX RDriveV1 = new VictorSPX(2); //1st Right Drive Motor
  private VictorSPX RDriveV2 = new VictorSPX(1); //2nd Right Drive Motor
  //private VictorSPX Intake = new VictorSPX(); //Intake Motor
  //private VictorSPX ConveyorBelt = new VictorSPX(); //Conveyor Belt Motor
  //private VictorSPX Flywheel = new VictorSPX(); //Flywheel Motor
  private XboxController xbox; // XBOX Controller
  private final I2C.Port i2cPort = I2C.Port.kOnboard; //Color Sensor Port Object
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort); //Color Sensor Object
  private final ColorMatch m_colorMatcher = new ColorMatch(); //Color Matcher Object (matches Colors with Color Sensor Output)
  private final Color blue = Color.kBlue; //The Color Blue
  private final Color red = Color.kRed; //The Color Red
  private Counter m_encoder_1; //Left Wheel Rotation Counter
  private Counter m_encoder_2; //Right Wheel Rotation Counter
  //private String previousColor = ""; //useless atm
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    //js1 = new Joystick(0);
    //js2 = new Joystick(1);
    xbox = new XboxController(2);//XBOX Controller is the 3rd controller after the 2 joysticks
    m_colorMatcher.addColorMatch(blue); //adds the blue profile to the color matcher
    m_colorMatcher.addColorMatch(red); // adds the red profile to the color matcher
    m_encoder_1 = new Counter(); // instantiates a new counter for the first encoder
    m_encoder_1.setUpSource(2);
    m_encoder_1.setUpDownCounterMode();
    m_encoder_1.setDistancePerPulse(Math.PI*6); //distance per pulse PI*diameter of wheel (6" in this case)
    m_encoder_1.setMaxPeriod(.2); //max period to determine if stopped
    m_encoder_1.reset();
    m_encoder_2 = new Counter(); //instantiates a new counter for the second encoder
    m_encoder_2.setUpSource(3);
    m_encoder_2.setUpDownCounterMode();
    m_encoder_2.setDistancePerPulse(Math.PI*6); //distance per pulse PI*diameter of wheel (6" in this case)
    m_encoder_2.setMaxPeriod(.2); //max period to determine if stopped
    m_encoder_2.reset();
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
    int count1= m_encoder_1.get(); //get the number of encoder counts
    double dist1 = m_encoder_1.getDistance(); //get the distance the wheel has spun
    double rate1 = m_encoder_1.getRate(); //get the rate at which the wheel is spinning
    SmartDashboard.putNumber("Encoder Counts", count1);
    SmartDashboard.putNumber("Distance", dist1);
    SmartDashboard.putNumber("Velocity (in/s)", rate1);
    int count2= m_encoder_2.get(); //get the number of encoder counts
    double dist2 = m_encoder_2.getDistance(); //get the distance the wheel has spun
    double rate2 = m_encoder_2.getRate(); //get the rate at which the wheel is spinning
    SmartDashboard.putNumber("Encoder Counts", count2);
    SmartDashboard.putNumber("Distance", dist2);
    SmartDashboard.putNumber("Velocity (in/s)", rate2);
    Color detectedColor = m_colorSensor.getColor();
    String colorString; 
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);  
    if (match.color == blue) {
      colorString = "Blue";
    } else if (match.color == red) {
      colorString = "Red";
    } else {
      colorString = "Unknown";
    }
    System.out.println(colorString);
    //previousColor = colorString;
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
    System.out.println("Auto selected: " + m_autoSelected);
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
    if(xbox.getRawAxis(2) >= 0.5){ //checks if trigger is pressed more than halfway (just in case u only press it a lil on accident)
      LDriveV1.set(ControlMode.PercentOutput, .96*xbox.getRawAxis(1));
      LDriveV2.set(ControlMode.PercentOutput, .96*xbox.getRawAxis(1));
    }if(xbox.getRawAxis(2) < 0.5){ //checks if trigger is not pressed more than halfway
      LDriveV1.set(ControlMode.PercentOutput, 0);
      LDriveV2.set(ControlMode.PercentOutput, 0);
    }
    if(xbox.getRawAxis(3) >= 0.5){// checks if trigger is pressed more than halfway
      RDriveV1.set(ControlMode.PercentOutput, 1*xbox.getRawAxis(5));
      RDriveV2.set(ControlMode.PercentOutput, 1*xbox.getRawAxis(5));
    }if(xbox.getRawAxis(3) < 0.5){// checks if trigger is not pressed more than halfway
      RDriveV1.set(ControlMode.PercentOutput, 0);
      RDriveV2.set(ControlMode.PercentOutput, 0);
    }

    //intake controllers
    if(xbox.getAButtonPressed()){
      //turn on
    }
    if(xbox.getBButtonPressed()){
      //turn off
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
}
