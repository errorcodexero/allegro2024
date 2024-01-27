package frc.robot.automodes;

import org.xero1425.base.actions.DelayAction;
import org.xero1425.base.controllers.AutoController;
import org.xero1425.base.controllers.SwerveTestAutoMode;
import org.xero1425.base.subsystems.motorsubsystem.MCMotionMagicAction;
import org.xero1425.base.subsystems.motorsubsystem.MCPositionAction;
import org.xero1425.base.subsystems.motorsubsystem.MCVelocityAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorPowerSequenceAction;

import frc.robot.subsystems.intakeshooter.StartCollectAction;
import frc.robot.subsystems.intakeshooter.StopCollectAction;
import frc.robot.subsystems.intakeshooter.StowAction;
import frc.robot.subsystems.intakeshooter.TransferNoteAction;
import frc.robot.subsystems.intakeshooter.TurtleAction;
import frc.robot.subsystems.targettracker.TargetTracker;
import frc.robot.subsystems.intakeshooter.EjectAction;
import frc.robot.subsystems.intakeshooter.IntakeShooterSubsystem;
import frc.robot.subsystems.intakeshooter.PrepareToShootAction;
import frc.robot.subsystems.intakeshooter.PrepareTransferNoteAction;
import frc.robot.subsystems.intakeshooter.ShootAction;
import frc.robot.subsystems.toplevel.AllegroRobot2024;

public class AllegroTestAutoMode extends SwerveTestAutoMode {

    public AllegroTestAutoMode(AutoController ctrl) throws Exception {
        super(ctrl, "Allegro-Test-Mode");

        AllegroRobot2024 robot = (AllegroRobot2024)ctrl.getRobot().getRobotSubsystem() ;
        IntakeShooterSubsystem intake = robot.getIntakeShooterSubsystem() ;
        TargetTracker tt = robot.getTargetTracker() ;

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
                if (intake != null && intake.spinner_feeder() != null) {
                    addSubActionPair(intake.spinner_feeder(), new MotorEncoderPowerAction(intake.spinner_feeder(), getDouble("power"), getDouble("duration")), true) ;
                }
                break ;

            case 11:
                if (intake != null && intake.spinner_feeder() != null) {
                    double duration = getDouble("duration") ;
                    double [] times = new double[] { duration, duration, duration, duration, duration } ;
                    double [] powers = new double[] { 0.1, 0.3, 0.5, 0.7, 0.9} ;
                    addSubActionPair(intake.spinner_feeder(), new MotorPowerSequenceAction(intake.spinner_feeder(), times, powers), true) ;
                }
                break ;                

            case 12:
                if (intake != null && intake.spinner_feeder() != null) {
                    addSubActionPair(intake.spinner_feeder(), new MCVelocityAction(intake.spinner_feeder(), "pids:velocity", getDouble("velocity")), true);
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
                    double duration = getDouble("duration") ;
                    double [] times = new double[] { duration, duration, duration, duration, duration } ;
                    double [] powers = new double[] { 0.1, 0.3, 0.5, 0.7, 0.9} ;
                    addSubActionPair(intake.updown(), new MotorPowerSequenceAction(intake.updown(), times, powers), true) ;
                }
                break ;                   

            case 22:
                if (intake != null && intake.updown() != null) {
                    addSubActionPair(intake.updown(), new MCMotionMagicAction(intake.updown(), "pids:position", getDouble("position"), getDouble("pos-threshold"), getDouble("vel-threshold")), true) ;
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
                    addSubActionPair(intake, new StartCollectAction(intake), false) ;
                    addAction(new DelayAction(getAutoController().getRobot(), 5.0));
                    addSubActionPair(intake, new StopCollectAction(intake), true) ;
                }
                break ;

            case 61:
                if (intake != null) {
                    addSubActionPair(intake, new PrepareToShootAction(intake), true);
                    addAction(new DelayAction(getAutoController().getRobot(), 5.0));
                    addSubActionPair(intake, new ShootAction(intake, tt), true);                    
                }
                break ;

            case 62:
                if (intake != null) {
                    addSubActionPair(intake, new PrepareTransferNoteAction(intake), true);
                    addAction(new DelayAction(getAutoController().getRobot(), 5.0));
                    addSubActionPair(intake, new TransferNoteAction(intake), true);

                }
                break ;

            case 63:
                if (intake != null) {
                    addSubActionPair(intake, new EjectAction(intake), true);
                }
                break ;      
                
            case 64:
                if (intake != null) {
                    addSubActionPair(intake, new StowAction(intake), true);
                }
                break ;       
                
            case 65:
                if (intake != null) {
                    addSubActionPair(intake, new TurtleAction(intake), true);
                }
                break ;                      
        }
    }
}
