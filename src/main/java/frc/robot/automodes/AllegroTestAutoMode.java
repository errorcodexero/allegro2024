package frc.robot.automodes;

import org.xero1425.base.actions.DelayAction;
import org.xero1425.base.controllers.AutoController;
import org.xero1425.base.controllers.SwerveTestAutoMode;
import org.xero1425.base.subsystems.motorsubsystem.MCMotionMagicAction;
import org.xero1425.base.subsystems.motorsubsystem.MCVelocityAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderSubsystem;
import org.xero1425.base.subsystems.motorsubsystem.MotorPowerSequenceAction;
import org.xero1425.base.subsystems.swerve.SwerveBaseSubsystem;
import org.xero1425.base.subsystems.swerve.SwerveHolonomicDynamicPathAction;
import org.xero1425.base.utils.Pose2dWithRotation;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.amp_trap.AmpTrapMoveNote;
import frc.robot.subsystems.amp_trap.AmpTrapPositionAction;
import frc.robot.subsystems.amp_trap.AmpTrapSubsystem;
import frc.robot.subsystems.intake_shooter.IntakeGotoNamedPositionAction;
import frc.robot.subsystems.intake_shooter.IntakeShooterStowAction;
import frc.robot.subsystems.intake_shooter.IntakeShooterSubsystem;
import frc.robot.subsystems.intake_shooter.ShooterTuningAction;
import frc.robot.subsystems.superstructure.ClimbAction;
import frc.robot.subsystems.superstructure.StowAction;
import frc.robot.subsystems.superstructure.SuperStructureSubsystem;
import frc.robot.subsystems.superstructure.TransferIntakeToTrampAction;
import frc.robot.subsystems.superstructure.ClimbAction.ClimbType;
import frc.robot.subsystems.toplevel.AllegroRobot2024;

public class AllegroTestAutoMode extends SwerveTestAutoMode {

    public AllegroTestAutoMode(AutoController ctrl) throws Exception {
        super(ctrl, "Allegro-Test-Mode");

        AllegroRobot2024 robot = (AllegroRobot2024) ctrl.getRobot().getRobotSubsystem();
        SuperStructureSubsystem superstructure = robot.getSuperStructure();
        IntakeShooterSubsystem intakeshooter = robot.getIntakeShooter();
        MotorEncoderSubsystem tilt = intakeshooter.getTilt();
        MotorEncoderSubsystem updown = intakeshooter.getUpDown();
        AmpTrapSubsystem amptrap = robot.getAmpTrap();
        MotorEncoderSubsystem climber = superstructure.getClimber();
        SwerveBaseSubsystem swerve = robot.getSwerve() ;

        if (createTest()) {
            //
            // If this returns true, the test mode was created for the swerve drive related
            // tests
            //
            return;
        }

        switch (getTestNumber()) {
            //
            // Note, test numbers 0 - 9 are in the base class and relate to the
            // swerve drive.
            //

            /////////////////////////////////////////////////////////////////////////
            //
            // Feeder motor tests
            //
            /////////////////////////////////////////////////////////////////////////
            case 10:
                if (intakeshooter != null) {
                    addSubActionPair(intakeshooter.getFeeder(), new MotorEncoderPowerAction(intakeshooter.getFeeder(),
                            getDouble("power"), getDouble("duration")), true);
                }
                break;

            case 11:
                if (intakeshooter != null && intakeshooter.getFeeder() != null) {
                    double duration = getDouble("duration");
                    double[] times = new double[] { duration, duration, duration, duration, duration };
                    double[] powers = new double[] { 0.1, 0.3, 0.5, 0.7, 0.9 };
                    addSubActionPair(intakeshooter.getFeeder(),
                            new MotorPowerSequenceAction(intakeshooter.getFeeder(), times, powers), true);
                }
                break;

            case 12:
                if (intakeshooter != null && intakeshooter.getFeeder() != null) {
                    addSubActionPair(intakeshooter.getFeeder(),
                            new MCVelocityAction(intakeshooter.getFeeder(), "pids:velocity", getDouble("velocity"), 1.0, true),
                            true);
                }
                break;

            /////////////////////////////////////////////////////////////////////////
            //
            // Updown motor tests
            //
            /////////////////////////////////////////////////////////////////////////
            case 20:
                if (intakeshooter != null) {
                    addSubActionPair(updown, new MotorEncoderPowerAction(updown, getDouble("power"), getDouble("duration")), true);
                }
                break;

            case 21:
                if (intakeshooter != null) {
                    double duration = getDouble("duration");
                    double[] times = new double[] { duration, duration, duration, duration, duration };
                    double[] powers = new double[] { 0.1, 0.3, 0.5, 0.7, 0.9 };
                    addSubActionPair(updown, new MotorPowerSequenceAction(updown, times, powers), true);
                }
                break;

            case 22:
                if (intakeshooter != null) {
                    addSubActionPair(tilt, new MCMotionMagicAction(tilt, "pids:position", getDouble("tilt-target"), 1.0, 1.0), true) ;
                    addSubActionPair(updown,new MCMotionMagicAction(updown, "pids:position", getDouble("updown-target"), 1.0, 1.0), true) ;
                }
                break;

            /////////////////////////////////////////////////////////////////////////
            //
            // Shooter1 motor tests
            //
            /////////////////////////////////////////////////////////////////////////
            case 30:
                if (intakeshooter != null && intakeshooter.getShooter1() != null) {
                    addSubActionPair(intakeshooter.getShooter1(), new MotorEncoderPowerAction(intakeshooter.getShooter1(),
                            getDouble("power"), getDouble("duration")), true);
                    addSubActionPair(intakeshooter.getShooter2(), new MotorEncoderPowerAction(intakeshooter.getShooter2(),
                            getDouble("power"), getDouble("duration")), true);                            
                }
                break;

            case 31:
                if (intakeshooter != null && intakeshooter.getShooter1() != null) {
                    double duration = getDouble("duration");
                    double[] times ;
                    double[] powers = new double[] { 0.1, 0.3, 0.5, 0.7 };
                    times = new double[powers.length] ;
                    for(int i = 0 ; i < powers.length ; i++) {
                        times[i] = duration ;
                    }
                    addSubActionPair(intakeshooter.getShooter1(),
                            new MotorPowerSequenceAction(intakeshooter.getShooter1(), times, powers), true);
                }
                break;

            case 32:
                if (intakeshooter != null) {
                    // addSubActionPair(intakeshooter.getShooter1(),
                    //         new MCVelocityAction(intakeshooter.getShooter1(), "pids:velocity", getDouble("velocity"), 1.0, true),
                    //         false);
                    addSubActionPair(intakeshooter.getShooter2(),
                            new MCVelocityAction(intakeshooter.getShooter2(), "pids:velocity", getDouble("velocity"), 1.0, true),
                            true);                            
                }
                break;

            /////////////////////////////////////////////////////////////////////////
            //
            // Shooter2 motor tests
            //
            /////////////////////////////////////////////////////////////////////////
            case 40:
                if (intakeshooter != null && intakeshooter.getShooter2() != null) {
                    addSubActionPair(intakeshooter.getShooter2(), new MotorEncoderPowerAction(intakeshooter.getShooter2(),
                            getDouble("power"), getDouble("duration")), true);
                }
                break;

            case 41:
                if (intakeshooter != null && intakeshooter.getShooter2() != null) {
                    double duration = getDouble("duration");
                    double[] times ;
                    double[] powers = new double[] { 0.1, 0.3, 0.5, 0.7 };
                    times = new double[powers.length] ;
                    for(int i = 0 ; i < powers.length ; i++) {
                        times[i] = duration ;
                    }
                    addSubActionPair(intakeshooter.getShooter2(),
                            new MotorPowerSequenceAction(intakeshooter.getShooter2(), times, powers), true);
                }
                break;

            case 42:
                if (intakeshooter != null && intakeshooter.getShooter2() != null) {
                    addSubActionPair(intakeshooter.getShooter2(),
                            new MCVelocityAction(intakeshooter.getShooter2(), "pids:velocity", getDouble("velocity"), 1.0, true),
                            true);
                }
                break;

            /////////////////////////////////////////////////////////////////////////
            //
            // Tilt motor tests
            //
            /////////////////////////////////////////////////////////////////////////
            case 50:
                if (intakeshooter != null && tilt != null) {
                    addSubActionPair(tilt, new MotorEncoderPowerAction(tilt,
                            getDouble("power"), getDouble("duration")), true);
                }
                break;

            case 51:
                if (intakeshooter != null) {
                    double duration = getDouble("duration");
                    double[] times = new double[] { duration, duration, duration, duration, duration };
                    double[] powers = new double[] { 0.1, 0.3, 0.5, 0.7, 0.9 };
                    addSubActionPair(tilt,
                            new MotorPowerSequenceAction(tilt, times, powers), true);
                }
                break;

            case 52:
                if (intakeshooter != null) {
                    addSubActionPair(tilt,
                            new MCMotionMagicAction(tilt, "pids:position", getDouble("target"), 
                                                        1, 1), true);
                }
                break;

            case 53:
                if (intakeshooter != null) {
                    addSubActionPair(tilt,
                            new MCMotionMagicAction(tilt, "pids:position", getDouble("target1"), 
                                                        3, 3), true);
                    addAction(new DelayAction(intakeshooter.getRobot(), 2.0));
                    addSubActionPair(tilt,
                            new MCMotionMagicAction(tilt, "pids:position", getDouble("target2"), 
                                                        3, 3), true);       
                    addAction(new DelayAction(intakeshooter.getRobot(), 2.0));
                    addSubActionPair(tilt,
                            new MCMotionMagicAction(tilt, "pids:position", getDouble("target3"), 
                                                        3, 3), true);
                    addAction(new DelayAction(intakeshooter.getRobot(), 2.0));
                    addSubActionPair(tilt,
                            new MCMotionMagicAction(tilt, "pids:position", getDouble("target4"), 
                                                        3, 3), true);                                                        
                }
                break;    

            /////////////////////////////////////////////////////////////////////////
            //
            // Elevator motor tests
            //
            /////////////////////////////////////////////////////////////////////////
            case 60:
                if (amptrap != null && amptrap.getElevator() != null) {
                    addSubActionPair(amptrap.getElevator(), new MotorEncoderPowerAction(amptrap.getElevator(),
                            getDouble("power"), getDouble("duration")), true);
                }
                break;

            case 61:
                if (amptrap != null && amptrap.getElevator() != null) {
                    double duration = getDouble("duration");
                    double[] times = new double[] { duration, duration, duration, duration, duration };
                    double[] powers = new double[] { 0.1, 0.3, 0.5, 0.7, 0.9 };
                    addSubActionPair(amptrap.getElevator(),
                            new MotorPowerSequenceAction(amptrap.getElevator(), times, powers), true);
                }
                break;

            case 62:
                if (amptrap != null) {
                    addSubActionPair(amptrap.getElevator(),
                            new MCMotionMagicAction(amptrap.getElevator(), "pids:position", 
                                getDouble("position"), .03, 0.05), true) ;
                }
                break;

            /////////////////////////////////////////////////////////////////////////
            //
            // Arm motor tests
            //
            /////////////////////////////////////////////////////////////////////////
            case 70:
                if (amptrap != null) {
                    addSubActionPair(amptrap.getArm(),
                            new MotorEncoderPowerAction(amptrap.getArm(), getDouble("power"), getDouble("duration")),
                            true);
                }
                break;

            case 71:
                if (amptrap != null && amptrap.getArm() != null) {
                    double duration = getDouble("duration");
                    double[] times = new double[] { duration, duration, duration, duration, duration };
                    double[] powers = new double[] { 0.1, 0.3, 0.5, 0.7, 0.9 };
                    addSubActionPair(amptrap.getArm(), new MotorPowerSequenceAction(amptrap.getArm(), times, powers),
                            true);
                }
                break;

            case 72:
                if (amptrap != null && amptrap.getArm() != null) {
                    addSubActionPair(amptrap.getArm(),
                            new MCMotionMagicAction(amptrap.getArm(), "pids:position", getDouble("target"), 5,5), true) ;
                }
                break;

            /////////////////////////////////////////////////////////////////////////
            //
            // Manipulator motor tests
            //
            /////////////////////////////////////////////////////////////////////////
            case 90:
                if (amptrap != null && amptrap.getManipulator() != null) {
                    addSubActionPair(amptrap.getManipulator(), new MotorEncoderPowerAction(amptrap.getManipulator(),
                            getDouble("power"), getDouble("duration")), true);
                }
                break;

            case 91:
                if (amptrap != null && amptrap.getManipulator() != null) {
                    double duration = getDouble("duration");
                    double[] times = new double[] { duration, duration, duration, duration, duration };
                    double[] powers = new double[] { 0.1, 0.3, 0.5, 0.7, 0.9 };
                    addSubActionPair(amptrap.getManipulator(),
                            new MotorPowerSequenceAction(amptrap.getManipulator(), times, powers), true);
                }
                break;

            case 92:
                if (amptrap != null && amptrap.getManipulator() != null) {
                    addSubActionPair(amptrap.getManipulator(),
                            new MCVelocityAction(amptrap.getManipulator(), "pids:velocity", getDouble("velocity"), 1.0, true),
                            true);
                }
                break;

            /////////////////////////////////////////////////////////////////////////
            //
            // Climber motor tests
            //
            ///////////////////////////////////////////////////////////////////////// '
            case 100:
                if (superstructure.getClimber() != null) {
                    addSubActionPair(amptrap.getElevator(),
                            new MCMotionMagicAction(amptrap.getElevator(), "pids:position", 
                                0.304, 0.1, 0.1), true) ;

                   addSubActionPair(amptrap.getArm(),
                            new MCMotionMagicAction(amptrap.getArm(), "pids:position", 120.0, 
                            5,5), true) ;

                    addSubActionPair(superstructure.getClimber(), new MotorEncoderPowerAction(superstructure.getClimber(),
                            getDouble("power"), getDouble("duration")), true);
                }
                break;

            case 101:
                if (superstructure.getClimber() != null) {
                    double duration = getDouble("duration");
                    double[] times = new double[] { duration, duration, duration, duration, duration };
                    double[] powers = new double[] { 0.1, 0.3, 0.5, 0.7, 0.9 };
                    addSubActionPair(superstructure.getClimber(),
                            new MotorPowerSequenceAction(superstructure.getClimber(), times, powers), true);
                }
                break;

            case 102:
                if (climber != null) {
                    addSubActionPair(amptrap.getElevator(),
                            new MCMotionMagicAction(amptrap.getElevator(), "pids:position", 
                                0.304, 0.1, 0.1), true) ;

                   addSubActionPair(amptrap.getArm(),
                            new MCMotionMagicAction(amptrap.getArm(), "pids:position", 120.0, 
                            5,5), true) ;

                    addSubActionPair(climber, new ClimbAction(climber, ClimbType.HooksUp), true) ;

                    addAction(new DelayAction(superstructure.getRobot(), 2.0));

                    addSubActionPair(climber, new ClimbAction(climber, ClimbType.HooksDown), true) ;

                }
                break;

            /////////////////////////////////////////////////////////////////////////
            //
            // Intake-Shooter tests
            //
            // This is for testing the intake shooter as a whole instead of its various
            // parts
            //
            /////////////////////////////////////////////////////////////////////////
            case 110:
                if (intakeshooter != null) {
                    addSubActionPair(intakeshooter, new IntakeShooterStowAction(intakeshooter), true);
                }
                break;

            case 111:
                break;

            case 118:
                if (intakeshooter != null) {
                    double v = getDouble("up-down") ;
                    double i = getDouble("tilt") ;
                    addSubActionPair(intakeshooter, new ShooterTuningAction(intakeshooter, v, i), true);
                }
                break ;

            case 119:
                if (intakeshooter != null) {
                    double shpower = getDouble("shooter-power") ;
                    double fdpower = getDouble("feeder-power") ;
                    addSubActionPair(intakeshooter.getShooter1(), new MotorEncoderPowerAction(intakeshooter.getShooter1(), shpower), false) ;
                    addSubActionPair(intakeshooter.getShooter2(), new MotorEncoderPowerAction(intakeshooter.getShooter2(), shpower), false) ;
                    addSubActionPair(intakeshooter.getFeeder(), new MotorEncoderPowerAction(intakeshooter.getFeeder(), fdpower), false) ;
                    addAction(new DelayAction(getAutoController().getRobot(), 30));
                }
                break ;


            /////////////////////////////////////////////////////////////////////////
            //
            // Amp-Trap tests
            //
            // This is for testing the amp-trap as a whole instead of its various parts
            //
            /////////////////////////////////////////////////////////////////////////
            case 120:
                if (amptrap != null) {
                    addSubActionPair(amptrap, new AmpTrapPositionAction(amptrap, getDouble("angle"), getDouble("height")), true) ;
                    addAction(new DelayAction(getAutoController().getRobot(), 5.0));
                    addSubActionPair(amptrap, new AmpTrapPositionAction(amptrap, 0.0, 0.0), true) ;
                }   
                break;

            case 121:
                if (amptrap != null) {
                    addSubActionPair(amptrap, new AmpTrapMoveNote(amptrap, "targets:trap"), true);
                }
                break ;

            /////////////////////////////////////////////////////////////////////////
            //
            // Superstructure tests
            //
            // This is for testing the amp-trap as a whole instead of its various parts
            //
            /////////////////////////////////////////////////////////////////////////            
            case 140:
                if (superstructure != null) {
                    addSubActionPair(superstructure, new TransferIntakeToTrampAction(robot.getSuperStructure()), true) ;
                }
                break ;

            case 141:
                if (superstructure != null) {
                    addSubActionPair(superstructure, new StowAction(superstructure), true) ;
                }
                break ;

            case 142:
                addSubActionPair(amptrap, new AmpTrapMoveNote(amptrap, -0.001), true) ;
                break ;

            case 143:
                addSubActionPair(amptrap, new AmpTrapPositionAction(amptrap, 225, 0.2), true);
                addAction(new DelayAction(amptrap.getRobot(), 2.0)) ;
                addSubActionPair(amptrap, new AmpTrapPositionAction(amptrap, 0.0, 0.05), true);        
                break ;       

            case 144:
                addSubActionPair(intakeshooter, new IntakeGotoNamedPositionAction(intakeshooter, 100, -55), true) ;
                addAction(new DelayAction(superstructure.getRobot(), 2.0));
                addSubActionPair(intakeshooter, new IntakeGotoNamedPositionAction(intakeshooter, -5, 50), true) ;
                break ;

            case 150:
                {
                    Pose2dWithRotation pts[] = new Pose2dWithRotation[2] ;
                    pts[0] = new Pose2dWithRotation(15.081, 5.517, Rotation2d.fromDegrees(180.0), Rotation2d.fromDegrees(180.0)) ;
                    pts[1] = new Pose2dWithRotation(14.041, 6.32, Rotation2d.fromDegrees(-135.0), Rotation2d.fromDegrees(135.0)) ;
                    SwerveHolonomicDynamicPathAction act = new SwerveHolonomicDynamicPathAction(swerve, "TestPath", 2.0, 1.5, 0.2, pts) ;
                    addSubActionPair(swerve, act, true);
                }
                break ;
        }
    }
}
