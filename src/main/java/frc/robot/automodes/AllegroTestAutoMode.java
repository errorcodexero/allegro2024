package frc.robot.automodes;

import org.xero1425.base.controllers.AutoController;
import org.xero1425.base.controllers.SwerveTestAutoMode;
import org.xero1425.base.subsystems.motorsubsystem.MCMotionMagicAction;
import org.xero1425.base.subsystems.motorsubsystem.MCVelocityAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorPowerSequenceAction;

import frc.robot.subsystems.amp_trap.AmpTrapStowAction;
import frc.robot.subsystems.amp_trap.AmpTrapSubsystem;
import frc.robot.subsystems.amp_trap.ClimbAction;

import frc.robot.subsystems.amp_trap.PrepClimbAction;
import frc.robot.subsystems.intake_shooter.IntakeShooterStowAction;

import frc.robot.subsystems.amp_trap.PrepTrapAction;
import frc.robot.subsystems.intake_shooter.IntakeShooterSubsystem;
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
                    addSubActionPair(intakeshooter.getUpDown(), new MCMotionMagicAction(intakeshooter.getUpDown(), "upDown", getDouble("target"), 0.5, 0.5), true);
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
                    addSubActionPair(intakeshooter.getTilt(), new MCMotionMagicAction(intakeshooter.getTilt(), "upDown", getDouble("target"), 0.5, 0.5), true);
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
                if (amptrap != null && amptrap.getArm() != null) {
                    addSubActionPair(amptrap.getArm(), new MotorEncoderPowerAction(amptrap.getArm(), getDouble("power"), getDouble("duration")), true) ;
                }
                break ;

            case 71:
                if (amptrap != null && amptrap.getArm() != null) {
                    double duration = getDouble("duration") ;
                    double [] times = new double[] { duration, duration, duration, duration, duration } ;
                    double [] powers = new double[] { 0.1, 0.3, 0.5, 0.7, 0.9} ;
                    addSubActionPair(amptrap.getArm(), new MotorPowerSequenceAction(amptrap.getArm(), times, powers), true) ;
                }
                break ;                

            case 72:
                if (amptrap != null && amptrap.getArm() != null) {
                    addSubActionPair(amptrap.getArm(), new MCVelocityAction(amptrap.getArm(), "pids:velocity", getDouble("velocity")), true);
                }
                break ;            
                
            /////////////////////////////////////////////////////////////////////////
            //
            // Wrists tests
            //
            /////////////////////////////////////////////////////////////////////////
            case 80:
                if (amptrap != null && amptrap.getWrist() != null) {
                    addSubActionPair(amptrap.getWrist(), new MotorEncoderPowerAction(amptrap.getWrist(), getDouble("power"), getDouble("duration")), true) ;
                }
                break ;

            case 81:
                if (amptrap != null && amptrap.getWrist() != null) {
                    double duration = getDouble("duration") ;
                    double [] times = new double[] { duration, duration, duration, duration, duration } ;
                    double [] powers = new double[] { 0.1, 0.3, 0.5, 0.7, 0.9} ;
                    addSubActionPair(amptrap.getWrist(), new MotorPowerSequenceAction(amptrap.getWrist(), times, powers), true) ;
                }
                break ;                

            case 82:
                if (amptrap != null && amptrap.getWrist() != null) {
                    addSubActionPair(amptrap.getWrist(), new MCVelocityAction(amptrap.getWrist(), "pids:velocity", getDouble("velocity")), true);
                }
                break ;                   

            /////////////////////////////////////////////////////////////////////////
            //
            // Manipulator tests
            //
            /////////////////////////////////////////////////////////////////////////
            case 90:
                if (amptrap != null && amptrap.getManipulator() != null) {
                    addSubActionPair(amptrap.getManipulator(), new MotorEncoderPowerAction(amptrap.getManipulator(), getDouble("power"), getDouble("duration")), true) ;
                }
                break ;

            case 91:
                if (amptrap != null && amptrap.getManipulator() != null) {
                    double duration = getDouble("duration") ;
                    double [] times = new double[] { duration, duration, duration, duration, duration } ;
                    double [] powers = new double[] { 0.1, 0.3, 0.5, 0.7, 0.9} ;
                    addSubActionPair(amptrap.getManipulator(), new MotorPowerSequenceAction(amptrap.getManipulator(), times, powers), true) ;
                }
                break ;                

            case 92:
                if (amptrap != null && amptrap.getManipulator() != null) {
                    addSubActionPair(amptrap.getManipulator(), new MCVelocityAction(amptrap.getManipulator(), "pids:velocity", getDouble("velocity")), true);
                }
                break ;     

            /////////////////////////////////////////////////////////////////////////
            //
            // Climber tests
            //
            /////////////////////////////////////////////////////////////////////////'
            case 100: 
                if (amptrap != null && amptrap.getClimber() != null) {
                    addSubActionPair(amptrap.getClimber(), new MotorEncoderPowerAction(amptrap.getClimber(), getDouble("power"), getDouble("duration")), true) ;
                }
                break ;

            case 101:
                if (amptrap != null && amptrap.getClimber() != null) {
                    double duration = getDouble("duration") ;
                    double [] times = new double[] { duration, duration, duration, duration, duration } ;
                    double [] powers = new double[] { 0.1, 0.3, 0.5, 0.7, 0.9} ;
                    addSubActionPair(amptrap.getClimber(), new MotorPowerSequenceAction(amptrap.getClimber(), times, powers), true) ;
                }
                break ;                

            case 102:
                if (amptrap != null && amptrap.getClimber() != null) {
                    addSubActionPair(amptrap.getClimber(), new MCMotionMagicAction(amptrap.getClimber(), "pids:position", 10, 0.5, 0.5), true);
                }
                break ;  
                
            /////////////////////////////////////////////////////////////////////////
            //
            // Intake-Shooter tests
            //
            /////////////////////////////////////////////////////////////////////////
            case 110:
                break;  
                
            case 111:
                break;

            /////////////////////////////////////////////////////////////////////////
            //
            // Amp-Trap tests
            //
            /////////////////////////////////////////////////////////////////////////
            case 120:
                if(amptrap != null && amptrap.getClimber() != null){
                    addSubActionPair(amptrap, new ClimbAction(amptrap), true);
                }
                break;  
                
            case 121:
                if(amptrap.getClimber() != null && amptrap.getElevator() != null && amptrap.getArm() != null && amptrap.getWrist() != null || intakeshooter != null && intakeshooter.getUpDown() != null && intakeshooter.getTilt() != null && intakeshooter.getFeeder() != null && intakeshooter.getShooter1() != null && intakeshooter.getShooter2() != null){
                    addSubActionPair(amptrap, new AmpTrapStowAction(amptrap), true);
                    addSubActionPair(intakeshooter, new IntakeShooterStowAction(intakeshooter), true);
                }
                break;   
            case 122:
                if(amptrap != null && amptrap.getClimber() != null && amptrap.getElevator() != null && amptrap.getArm() != null && amptrap.getWrist() != null){
                    addSubActionPair(amptrap, new PrepTrapAction(amptrap), true);
                }
                break;
                
            case 123:
                if(amptrap != null && amptrap.getClimber() != null && amptrap.getElevator() != null && amptrap.getArm() != null && amptrap.getWrist() != null){
                    addSubActionPair(amptrap, new PrepClimbAction(amptrap), true);
                }
        }
    }
}
