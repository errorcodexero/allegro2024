package frc.robot.automodes;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.swerve.SwerveHolonomicPathFollower;
import org.xero1425.base.subsystems.swerve.SwerveTrackAngle;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;

import frc.robot.subsystems.intake_shooter.IntakeAutoShootAction;
import frc.robot.subsystems.intake_shooter.IntakeGotoNamedPositionAction;
import frc.robot.subsystems.intake_shooter.StartCollectAltAction;
import frc.robot.subsystems.toplevel.AllegroRobot2024;

public class Start1Shoot3Action extends Action {
    private static final double kRotatePosTol = 5.0 ;

    private enum State {
        Idle,
        Shoot1,
        Path1,
        FinishCollect1,
        MissedCollect,
        MissedStow,
        Shoot2,
        Path2,
        Path3,
        Shoot3,
        Stowing,
        Done
    }

    private boolean mirror_ ;
    private double mvalue_ ;
    private AllegroRobot2024 robot_ ;
    private State state_ ;

    private SwerveTrackAngle rotate_ ;
    private IntakeAutoShootAction shoot_ ;
    private IntakeGotoNamedPositionAction stow_ ;
    private StartCollectAltAction start_collect_ ;
    private SwerveHolonomicPathFollower p1_ ;
    private SwerveHolonomicPathFollower p2_ ;  
    private SwerveHolonomicPathFollower p3_ ;

    public Start1Shoot3Action(AllegroRobot2024 robot, boolean mirror, double mvalue) throws Exception {
        super(robot.getRobot().getMessageLogger()) ;

        robot_ = robot ;
        mirror_ = mirror ;
        mvalue_ = mvalue ;
        state_ = State.Idle ;

        rotate_ = new SwerveTrackAngle(robot.getSwerve(), () -> robot_.getTargetTracker().getRotation(), kRotatePosTol) ;
        shoot_ = new IntakeAutoShootAction(robot_.getIntakeShooter(), robot_.getTargetTracker(), true, rotate_) ;
        double v1 = robot_.getIntakeShooter().getUpDown().getSettingsValue("targets:stow").getDouble() ;
        double v2 = robot_.getIntakeShooter().getTilt().getSettingsValue("targets:stow").getDouble() ; 
        stow_ = new IntakeGotoNamedPositionAction(robot_.getIntakeShooter(), v1, v2) ;
        start_collect_ = new StartCollectAltAction(robot_.getIntakeShooter()) ;

        p1_ = new SwerveHolonomicPathFollower(robot.getSwerve(), "S1S3-P1", true, 0.2, mirror_, mvalue_);
        p2_ = new SwerveHolonomicPathFollower(robot.getSwerve(), "S1S3-P2", false, 0.2, mirror_, mvalue_);
        p3_ = new SwerveHolonomicPathFollower(robot.getSwerve(), "S1S3-P3", false, 0.2, mirror_, mvalue_);
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        state_ = State.Shoot1 ;
        robot_.getSwerve().setAction(rotate_, true) ;
        robot_.getIntakeShooter().setAction(shoot_, true) ;
    }

    private void idleState() {        
    }

    private void doneState() {
        setDone() ;
    }

    private void shoot1State() {
        if (shoot_.isDone()) {
            robot_.getIntakeShooter().setAction(start_collect_, true) ;            
            robot_.getSwerve().setAction(p1_, true) ;
            state_ = State.Path1 ;
        }
    }

    private void path1State() {
        
        if (p1_.isDone()) {
            MessageLogger logger = robot_.getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Info).add("holding note", robot_.getIntakeShooter().isHoldingNote()).endMessage();
        }

        if (p1_.isDone() && robot_.getIntakeShooter().isHoldingNote()) {
            state_ = State.FinishCollect1 ;
        }
        else if (p1_.isDone()) {
            //
            // We missed the note, 
            //
            robot_.getIntakeShooter().setAction(stow_, true) ;
            state_ = State.MissedCollect ;
        }
    }

    private void finishCollect1State() {
        if (start_collect_.isDone()) {
            //
            // We got the note, shoot it
            //
            robot_.getSwerve().setAction(rotate_, true) ;
            robot_.getIntakeShooter().setAction(shoot_, true) ;                
            state_ = State.Shoot2 ;
        }
    }

    private void missedCollectState() {
        state_ = State.MissedStow ;
    }

    private void missedStowState() {
        if (stow_.isDone()) {
            state_ = State.Done ;
        }
    }

    private void shoot2State() {
        if (shoot_.isDone()) {
            robot_.getIntakeShooter().setAction(start_collect_, true) ;            
            robot_.getSwerve().setAction(p2_, true) ;
            state_ = State.Path2 ;            
        }
    }

    private void path2State() {
        if (p2_.isDone() && robot_.getIntakeShooter().isHoldingNote()) {
            //
            // We got the note, drive back
            //
            robot_.getSwerve().setAction(p3_, true) ;
            state_ = State.Path3 ;
        }
        else if (p2_.isDone()) {
            //
            // We missed the note, 
            //
            robot_.getIntakeShooter().setAction(stow_, true);            
            state_ = State.MissedCollect ;
        }            
    }
  
    private void path3State() {
        if (p3_.isDone()) {
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
            robot_.getIntakeShooter().setAction(stow_, true) ;
            state_ = State.Stowing ;
        }
    }

    private void stowingState() {
        if (stow_.isDone()) {
            state_ = State.Done;            
        }
    }    

    @Override
    public void run() throws Exception {
        super.run() ;

        State prev = state_ ;

        switch(state_) {
            case Idle:
                idleState() ;
                break ;

            case Done:
                doneState() ;
                break ;

            case Shoot1:
                shoot1State() ;
                break ;

            case Path1:
                path1State() ;
                break ;

            case FinishCollect1:
                finishCollect1State();
                break ;

            case MissedCollect:
                missedCollectState() ;
                break ;

            case MissedStow:
                missedStowState();
                break ;

            case Shoot2:
                shoot2State() ;
                break ;

            case Path2:
                path2State() ;
                break ;

            case Path3:
                path3State() ;
                break ;

            case Shoot3:
                shoot3State() ;
                break ;

            case Stowing:
                stowingState();
                break ;
        }

        if (prev != state_) {
            MessageLogger logger = robot_.getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Info);
            logger.add("Start1Shoot3Action action changed states: " + prev.toString() + " -> " + state_.toString()) ;
            logger.endMessage();
        }
    }

    @Override
    public String toString(int indent) {
        return prefix(indent) + "Start2Shoot3Action" ;
    }
}
