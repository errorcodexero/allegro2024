package frc.robot.automodes;

import org.xero1425.base.actions.DelayAction;
import org.xero1425.base.controllers.AutoController;
import org.xero1425.base.controllers.SwerveTestAutoMode;
import org.xero1425.base.subsystems.motorsubsystem.MCVelocityAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorPowerSequenceAction;

import frc.robot.subsystems.intake_shooter.IntakeShooterSubsystem;
import frc.robot.subsystems.intake_shooter.StartCollectAction;
import frc.robot.subsystems.intake_shooter.StopCollectAction;
import frc.robot.subsystems.intake_shooter.ShooterTuningAction;
import frc.robot.subsystems.ampTrap.AmpTrapSubsystem;
import frc.robot.subsystems.toplevel.AllegroRobot2024;

public class AllegroTestAutoMode extends SwerveTestAutoMode {

    public AllegroTestAutoMode(AutoController ctrl) throws Exception {
        super(ctrl, "Allegro-Test-Mode");

        AllegroRobot2024 robot = (AllegroRobot2024)ctrl.getRobot().getRobotSubsystem() ;
        IntakeShooterSubsystem intakeshooter = robot.getIntakeShooter();
        AmpTrapSubsystem amptrap = robot.getAmpTrap() ;

        if (createTest()) {
            //
            // If this returns true, the test mode was created for the swerve drive related tests
            //
            return ;
        }

        switch (getTestNumber()) {
            /////////////////////////////////////////////////////////////////////////
            //
            // Feeder tests
            //
            /////////////////////////////////////////////////////////////////////////
            case 10:
                if(intakeshooter != null && intakeshooter.getFeeder() != null) {
                    addSubActionPair(intakeshooter.getTilt(), new MotorEncoderPowerAction(intakeshooter.getFeeder(), getDouble("power"), getDouble("duration")), true);
                }
                break;   
                
            case 11: 
                if (intakeshooter != null && intakeshooter.getFeeder() != null) {
                    double duration = getDouble("duration") ;
                    double [] times = new double[] { duration, duration, duration, duration, duration } ;
                    double [] powers = new double[] { 0.1, 0.3, 0.5, 0.7, 0.9} ;
                    addSubActionPair(intakeshooter.getFeeder(), new MotorPowerSequenceAction(intakeshooter.getFeeder(), times, powers), true) ;
                }
                break;

            case 12:
                if (intakeshooter != null && intakeshooter.getFeeder() != null) {
                    addSubActionPair(intakeshooter.getFeeder(), new MCVelocityAction(intakeshooter.getFeeder(), "pids:velocity", getDouble("velocity")), true);
                }
                break; 

            /////////////////////////////////////////////////////////////////////////
            //
            // Updown tests
            //
            /////////////////////////////////////////////////////////////////////////
            case 20:
                if(intakeshooter != null && intakeshooter.getUpDown() != null) {
                    addSubActionPair(intakeshooter.getTilt(), new MotorEncoderPowerAction(intakeshooter.getUpDown(), getDouble("power"), getDouble("duration")), true);
                }
                break;   
                
            case 21: 
                if (intakeshooter != null && intakeshooter.getUpDown() != null) {
                    double duration = getDouble("duration") ;
                    double [] times = new double[] { duration, duration, duration, duration, duration } ;
                    double [] powers = new double[] { 0.1, 0.3, 0.5, 0.7, 0.9} ;
                    addSubActionPair(intakeshooter.getUpDown(), new MotorPowerSequenceAction(intakeshooter.getUpDown(), times, powers), true) ;
                }
                break;

            case 22:
                if (intakeshooter != null && intakeshooter.getUpDown() != null) {
                    addSubActionPair(intakeshooter.getUpDown(), new MCVelocityAction(intakeshooter.getUpDown(), "pids:velocity", getDouble("velocity")), true);
                }
                break; 

            /////////////////////////////////////////////////////////////////////////
            //
            // Shooter1 tests
            //
            /////////////////////////////////////////////////////////////////////////
            case 30:
                if(intakeshooter != null && intakeshooter.getShooter1() != null) {
                    addSubActionPair(intakeshooter.getTilt(), new MotorEncoderPowerAction(intakeshooter.getShooter1(), getDouble("power"), getDouble("duration")), true);
                }
                break;   
                
            case 31: 
                if (intakeshooter != null && intakeshooter.getShooter1() != null) {
                    double duration = getDouble("duration") ;
                    double [] times = new double[] { duration, duration, duration, duration, duration } ;
                    double [] powers = new double[] { 0.1, 0.3, 0.5, 0.7, 0.9} ;
                    addSubActionPair(intakeshooter.getShooter1(), new MotorPowerSequenceAction(intakeshooter.getShooter1(), times, powers), true) ;
                }
                break;

            case 32:
                if (intakeshooter != null && intakeshooter.getShooter1() != null) {
                    addSubActionPair(intakeshooter.getShooter1(), new MCVelocityAction(intakeshooter.getShooter1(), "pids:velocity", getDouble("velocity")), true);
                }
                break; 

            /////////////////////////////////////////////////////////////////////////
            //
            // Shooter2 tests
            //
            /////////////////////////////////////////////////////////////////////////
            case 40:
                if(intakeshooter != null && intakeshooter.getShooter2() != null) {
                    addSubActionPair(intakeshooter.getTilt(), new MotorEncoderPowerAction(intakeshooter.getShooter2(), getDouble("power"), getDouble("duration")), true);
                }
                break;   
                
            case 41: 
                if (intakeshooter != null && intakeshooter.getShooter2() != null) {
                    double duration = getDouble("duration") ;
                    double [] times = new double[] { duration, duration, duration, duration, duration } ;
                    double [] powers = new double[] { 0.1, 0.3, 0.5, 0.7, 0.9} ;
                    addSubActionPair(intakeshooter.getShooter2(), new MotorPowerSequenceAction(intakeshooter.getShooter2(), times, powers), true) ;
                }
                break;

            case 42:
                if (intakeshooter != null && intakeshooter.getShooter2() != null) {
                    addSubActionPair(intakeshooter.getShooter2(), new MCVelocityAction(intakeshooter.getShooter2(), "pids:velocity", getDouble("velocity")), true);
                }
                break;

            /////////////////////////////////////////////////////////////////////////
            //
            // Tilt tests
            //
            /////////////////////////////////////////////////////////////////////////
            case 50:
                if(intakeshooter != null && intakeshooter.getTilt() != null) {
                    addSubActionPair(intakeshooter.getTilt(), new MotorEncoderPowerAction(intakeshooter.getTilt(), getDouble("power"), getDouble("duration")), true);
                }
                break;   
                
            case 51: 
                if (intakeshooter != null && intakeshooter.getTilt() != null) {
                    double duration = getDouble("duration") ;
                    double [] times = new double[] { duration, duration, duration, duration, duration } ;
                    double [] powers = new double[] { 0.1, 0.3, 0.5, 0.7, 0.9} ;
                    addSubActionPair(intakeshooter.getTilt(), new MotorPowerSequenceAction(intakeshooter.getTilt(), times, powers), true) ;
                }
                break;

            case 52:
                if (intakeshooter != null && intakeshooter.getTilt() != null) {
                    addSubActionPair(intakeshooter.getTilt(), new MCVelocityAction(intakeshooter.getTilt(), "pids:velocity", getDouble("velocity")), true);
                }
                break;  
                
            /////////////////////////////////////////////////////////////////////////
            //
            // Elevator tests
            //
            /////////////////////////////////////////////////////////////////////////
            case 60:
                if (amptrap != null && amptrap.getElevator() != null) {
                    addSubActionPair(amptrap.getElevator(), new MotorEncoderPowerAction(amptrap.getElevator(), getDouble("power"), getDouble("duration")), true) ;
                }
                break ;

            case 61:
                if (amptrap != null && amptrap.getElevator() != null) {
                    double duration = getDouble("duration") ;
                    double [] times = new double[] { duration, duration, duration, duration, duration } ;
                    double [] powers = new double[] { 0.1, 0.3, 0.5, 0.7, 0.9} ;
                    addSubActionPair(amptrap.getElevator(), new MotorPowerSequenceAction(amptrap.getElevator(), times, powers), true) ;
                }
                break ;                

            case 62:
                if (amptrap != null && amptrap.getElevator() != null) {
                    addSubActionPair(amptrap.getElevator(), new MCVelocityAction(amptrap.getElevator(), "pids:velocity", getDouble("velocity")), true);
                }
                break ;   
                
            /////////////////////////////////////////////////////////////////////////
            //
            // Arm tests
            //
            /////////////////////////////////////////////////////////////////////////
            case 70:
                break;          
                
            /////////////////////////////////////////////////////////////////////////
            //
            // Wrists tests
            //
            /////////////////////////////////////////////////////////////////////////
            case 80:
                break;                 

            /////////////////////////////////////////////////////////////////////////
            //
            // Manipulator tests
            //
            /////////////////////////////////////////////////////////////////////////
            case 90:
                break;    
                
            /////////////////////////////////////////////////////////////////////////
            //
            // Intake-Shooter tests
            //
            /////////////////////////////////////////////////////////////////////////
            case 100:
                addSubActionPair(intakeshooter, new StartCollectAction(intakeshooter), true);
                addAction(new DelayAction(robot.getRobot(), 1.0));
                addSubActionPair(intakeshooter, new StopCollectAction(intakeshooter), true);
                break;

            case 101:
                addSubActionPair(intakeshooter, new ShooterTuningAction(intakeshooter), true);
                break;

            /////////////////////////////////////////////////////////////////////////
            //
            // Amp-Trap tests
            //
            /////////////////////////////////////////////////////////////////////////
            case 120:
                break;                 
        }
    }
}
