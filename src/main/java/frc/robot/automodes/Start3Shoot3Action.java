package frc.robot.automodes;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.swerve.SwerveHolonomicPathFollower;
import org.xero1425.base.subsystems.swerve.SwerveTrackAngle;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;

import frc.robot.subsystems.intake_shooter.IntakeAutoShootAction;
import frc.robot.subsystems.intake_shooter.IntakeGotoNamedPositionAction;
import frc.robot.subsystems.intake_shooter.IntakeManualShootAction;
import frc.robot.subsystems.intake_shooter.StartCollectAltAction;
import frc.robot.subsystems.intake_shooter.StopCollectAltAction;
import frc.robot.subsystems.toplevel.AllegroRobot2024;

public class Start3Shoot3Action extends Action {
    private static final double kRotatePosTol = 2.0 ;

    private enum State {
        Idle,
        Shoot1,
        Shoot2,
        Shoot3,
        Path1,
        Path2,
        Path3,
        Path4,
        MissedCollect,
        MissedStow,
        Done
    }

    private State state_ ;
    private boolean mirror_ ;
    private double mvalue_ ;
    private AllegroRobot2024 robot_ ;

    private SwerveHolonomicPathFollower p1_ ;    
    private SwerveHolonomicPathFollower p2_ ;    
    private SwerveHolonomicPathFollower p3_ ;
    private SwerveHolonomicPathFollower p4_ ;
    private IntakeManualShootAction manual_shoot_ ;    
    private SwerveTrackAngle rotate_ ;
    private IntakeAutoShootAction shoot_ ;    
    private StartCollectAltAction start_collect_ ;
    private StopCollectAltAction stop_collect_ ;    
    private IntakeGotoNamedPositionAction stow_ ;

    public Start3Shoot3Action(AllegroRobot2024 robot, boolean mirror, double mvalue) throws Exception {
        super(robot.getRobot().getMessageLogger()) ;

        robot_ = robot ;
        mirror_ = mirror ;
        mvalue_ = mvalue ;
        state_ = State.Idle ;

        rotate_ = new SwerveTrackAngle(robot.getSwerve(), () -> robot_.getTargetTracker().getRotation(), kRotatePosTol) ;
        shoot_ = new IntakeAutoShootAction(robot_.getIntakeShooter(), robot_.getTargetTracker(), true, rotate_) ;
        start_collect_ = new StartCollectAltAction(robot_.getIntakeShooter()) ;
        stop_collect_ = new StopCollectAltAction(robot_.getIntakeShooter()) ;

        double v1 = robot_.getIntakeShooter().getUpDown().getSettingsValue("targets:stow").getDouble() ;
        double v2 = robot_.getIntakeShooter().getTilt().getSettingsValue("targets:stow").getDouble() ; 
        stow_ = new IntakeGotoNamedPositionAction(robot_.getIntakeShooter(), v1, v2) ;     
        
        if (mirror) {
            manual_shoot_ = new IntakeManualShootAction(robot.getIntakeShooter(), "subwoofer-left") ;             
        }
        else {
            manual_shoot_ = new IntakeManualShootAction(robot_.getIntakeShooter(), "subwoofer-right") ;        
        }

        p1_ = new SwerveHolonomicPathFollower(robot.getSwerve(), "S3S3-P1", true, 0.2, mirror_, mvalue_);
        p2_ = new SwerveHolonomicPathFollower(robot.getSwerve(), "S3S3-P2", true, 0.2, mirror_, mvalue_); 
        p3_ = new SwerveHolonomicPathFollower(robot.getSwerve(), "S3S3-P3", true, 0.2, mirror_, mvalue_);
        p4_ = new SwerveHolonomicPathFollower(robot.getSwerve(), "S3S3-P4", true, 0.2, mirror_, mvalue_);        
    }

    @Override
    public void start() {
        state_ = State.Shoot1 ;
        robot_.getIntakeShooter().setAction(manual_shoot_, true) ;
    }

    private void shoot1State() {
        if (manual_shoot_.isDone()) {
            robot_.getIntakeShooter().setAction(start_collect_, true) ;
            robot_.getSwerve().setAction(p1_, true) ;
            state_ = State.Path1 ;
        }        
    }

    private void path1State() {
        if (p1_.isDone() && robot_.getIntakeShooter().isHoldingNote()) {
            //
            // We got the note, drive back
            //
            robot_.getSwerve().setAction(p2_, true) ;
            state_ = State.Path2 ;
        }
        else if (p1_.isDone()) {
            //
            // We missed the note, 
            //
            state_ = State.MissedCollect ;
        }
    }

    private void path2State() {
        if (p2_.isDone()) {
            //
            // We have arrived, shoot the note
            //
            robot_.getSwerve().setAction(rotate_, true) ;
            robot_.getIntakeShooter().setAction(shoot_, true) ;
            state_ = State.Shoot2 ;
        }
    }

    private void shoot2State() {
        if (shoot_.isDone()) {
            robot_.getIntakeShooter().setAction(start_collect_, true) ;            
            robot_.getSwerve().setAction(p3_, true) ;
            state_ = State.Path3 ;
        }
    }

    private void path3State() {
        if (p3_.isDone() && robot_.getIntakeShooter().isHoldingNote()) {
            //
            // We got the note, drive back
            //
            robot_.getSwerve().setAction(p4_, true) ;
            state_ = State.Path4 ;
        }
        else if (p3_.isDone()) {
            //
            // We missed the note, 
            //
            state_ = State.MissedCollect ;
        }
    }

    private void path4State() {
        if (p4_.isDone()) {
            //
            // We have arrived, shoot the note
            //
            robot_.getSwerve().setAction(rotate_, true) ;
            robot_.getIntakeShooter().setAction(shoot_, true) ;
            state_ = State.Shoot3 ;
        }        
    }

    private void shoot3State() {
        if (shoot_.isDone()) {
            //
            // This will stow the intake if possible (time)
            //
            state_ = State.MissedCollect ;
        }
    }

    private void missedCollect1State() {
        if (stop_collect_.isDone()) {
            //
            // We just stop here for now (maybe do more later)
            //
            robot_.getIntakeShooter().setAction(stow_, true) ;
            state_ = State.MissedStow ;
        }
    }

    private void missedStowState() {
        if (stow_.isDone()) {
            state_ = State.Done ;
        }
    }

    @Override
    public void run() {
        State prev = state_ ;

        switch(state_) {
            case Done:
                setDone() ;
                break ;
            
            case Idle:
                break ;
            
            case Shoot1:
                shoot1State() ;
                break ;

            case Shoot2:
                shoot2State() ;
                break ;

            case Shoot3:
                shoot3State() ;
                break ;                

            case Path1:
                path1State();
                break ;

            case Path2:
                path2State() ;
                break ;

            case Path3:
                path3State() ;
                break ;
                
            case Path4:
                path4State() ;
                break ;                

            case MissedCollect:
                missedCollect1State();
                break ;

            case MissedStow:
                missedStowState();
                break ;
        }

        if (state_ != prev) {
            MessageLogger logger = robot_.getRobot().getMessageLogger();
            logger.startMessage(MessageType.Info) ;
            logger.add("automode: " + prev.toString() + " -> " + state_.toString()) ;
            logger.endMessage();            
        }
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "Start3Shoot3Action" ;
    }
}
