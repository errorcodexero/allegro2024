package frc.robot.automodes;

import org.xero1425.base.controllers.AutoController;
import org.xero1425.base.controllers.SwerveTestAutoMode;
import org.xero1425.base.subsystems.motorsubsystem.MCPositionAction;
import org.xero1425.base.subsystems.motorsubsystem.MCVelocityAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorPowerSequenceAction;

import frc.robot.subsystems.intakeshooter.CollectAction;
import frc.robot.subsystems.intakeshooter.IntakeShooterSubsystem;
import frc.robot.subsystems.toplevel.AllegroRobot2024;

public class AllegroTestAutoMode extends SwerveTestAutoMode {

    public AllegroTestAutoMode(AutoController ctrl) throws Exception {
        super(ctrl, "Allegro-Test-Mode");

        AllegroRobot2024 robot = (AllegroRobot2024)ctrl.getRobot().getRobotSubsystem() ;
        IntakeShooterSubsystem intake = robot.getIntakeShooterSubsystem() ;

        if (createTest()) {
            //
            // If this returns true, the test mode was created for the swerve drive related tests
            //
            return ;
        }

        switch (getTestNumber()) {
            //////////////////////////////////////////////////////////////////////////////////////
            //
            // Intake test modes
            //
            //////////////////////////////////////////////////////////////////////////////////////

            // 
            // Spinner tests
            // 

            //
            // Basic test to just apply power - test to see that it is working
            //
            case 10:
                if (intake != null && intake.spinner() != null) {
                    addSubActionPair(intake.spinner(), new MotorEncoderPowerAction(intake.spinner(), getDouble("power"), getDouble("duration")), true) ;
                }
                break ;

            case 11:
                if (intake != null && intake.spinner() != null) {
                    double duration = getDouble("duration") ;
                    double [] times = new double[] { duration, duration, duration, duration, duration } ;
                    double [] powers = new double[] { 0.1, 0.3, 0.5, 0.7, 0.9} ;
                    addSubActionPair(intake.spinner(), new MotorPowerSequenceAction(intake.spinner(), times, powers), true) ;
                }
                break ;                

            case 12:
                if (intake != null && intake.spinner() != null) {
                    addSubActionPair(intake.spinner(), new MCVelocityAction(intake.spinner(), "pids:velocity", getDouble("velocity")), true);
                }
                break ;



            //
            // updown tests
            //
            case 20:
                if (intake != null && intake.updown() != null) {
                    addSubActionPair(intake.updown(), new MotorEncoderPowerAction(intake.updown(), getDouble("power"), getDouble("duration")), true) ;
                }
                break ;

            case 21:
                if (intake != null && intake.updown() != null) {
                    addSubActionPair(intake.updown(), new MCPositionAction(intake.updown(), "collect", getDouble("position")), true);
                }
                break ;                

            //
            // tilt tests
            //
            case 30:
                if (intake != null && intake.tilt() != null) {
                    addSubActionPair(intake.tilt(), new MotorEncoderPowerAction(intake.tilt(), getDouble("power"), getDouble("duration")), true) ;
                }
                break ;

            case 31:
                if (intake != null && intake.tilt() != null) {
                    addSubActionPair(intake.tilt(), new MCPositionAction(intake.tilt(), "collect", getDouble("position")), true);
                }
                break ;                   

            //
            // feeder tests
            //
            case 40:
                if (intake != null && intake.feeder() != null) {
                    addSubActionPair(intake.feeder(), new MotorEncoderPowerAction(intake.feeder(), getDouble("power"), getDouble("duration")), true) ;
                }
                break ;

            case 41:
                if (intake != null && intake.feeder() != null) {
                    double duration = getDouble("duration") ;
                    double [] times = new double[] { duration, duration, duration, duration, duration } ;
                    double [] powers = new double[] { 0.1, 0.3, 0.5, 0.7, 0.9} ;
                    addSubActionPair(intake.feeder(), new MotorPowerSequenceAction(intake.feeder(), times, powers), true) ;
                }
                break ;                  

            case 42:
                if (intake != null && intake.feeder() != null) {
                    addSubActionPair(intake.feeder(), new MCVelocityAction(intake.feeder(), "pids:velocity", getDouble("velocity")), true);
                }
                break ;                

            //
            // shooter tests
            //
            case 50:
                if (intake != null && intake.shooter1() != null) {
                    addSubActionPair(intake.shooter1(), new MotorEncoderPowerAction(intake.shooter1(), getDouble("power"), getDouble("duration")), true) ;
                }
                break ;  

            case 51:
                if (intake != null && intake.shooter1() != null) {
                    addSubActionPair(intake.shooter1(), new MCVelocityAction(intake.shooter1(), "shoot", getDouble("velocity")), true);
                }
                break ;  
                
            case 52:
                if (intake != null && intake.shooter2() != null) {
                    addSubActionPair(intake.shooter2(), new MotorEncoderPowerAction(intake.shooter2(), getDouble("power"), getDouble("duration")), true) ;
                }
                break ;  

            case 53:
                if (intake != null && intake.shooter2() != null) {
                    addSubActionPair(intake.shooter2(), new MCVelocityAction(intake.shooter2(), "shoot", getDouble("velocity")), true);
                }
                break ;                  

            //
            // complete intake-shooter tests
            //
            case 60:
                if (intake != null) {
                    addSubActionPair(intake, new CollectAction(intake), true) ;
                }
                break ;

            //
            // Elevator tests
            //
            case 70:
                break ;

            //
            // Pivot tests
            //
            case 80:
                break ;

            //
            // Manipulator tests
            //
            case 90:
                break ; 

            //
            // complete elevator-pivot-manipulator tests
            //
            case 100:
                break;
        }
    }
}
