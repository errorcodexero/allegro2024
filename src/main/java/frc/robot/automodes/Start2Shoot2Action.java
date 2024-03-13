package frc.robot.automodes;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.swerve.SwerveHolonomicPathFollower;
import org.xero1425.base.subsystems.swerve.SwerveTrackAngle;

import frc.robot.subsystems.intake_shooter.IntakeAutoShootAction;
import frc.robot.subsystems.intake_shooter.IntakeGotoNamedPositionAction;
import frc.robot.subsystems.intake_shooter.IntakeManualShootAction;
import frc.robot.subsystems.intake_shooter.StartCollectAltAction;
import frc.robot.subsystems.toplevel.AllegroRobot2024;

public class Start2Shoot2Action extends Action {
    private enum State {
        Idle,
        Shoot1,
        Shoot2,
        Path1,
        FinishCollect1,
        MissedCollect,
        MissedStow,
        Done
    }

    private AllegroRobot2024 robot_ ;
    private State state_ ;

    private IntakeManualShootAction manual_shoot_ ;
    private StartCollectAltAction start_collect_ ;    
    private SwerveHolonomicPathFollower p1_ ;    
    private SwerveTrackAngle rotate_ ;
    private IntakeAutoShootAction shoot_ ;
    private IntakeGotoNamedPositionAction stow_ ;    

    public Start2Shoot2Action(AllegroRobot2024 robot, boolean mirror, double mvalue) throws Exception {
        super(robot.getRobot().getMessageLogger()) ;
        robot_ = robot ;
        state_ = State.Idle ;

        double rottol = robot.getIntakeShooter().getSettingsValue("actions:auto-shoot:rotational-position-tolerance").getDouble() ;

        manual_shoot_ = new IntakeManualShootAction(robot_.getIntakeShooter(), "subwoofer-center") ;
        start_collect_ = new StartCollectAltAction(robot_.getIntakeShooter()) ;        
        rotate_ = new SwerveTrackAngle(robot.getSwerve(), () -> robot_.getTargetTracker().getRotation(), rottol) ;
        shoot_ = new IntakeAutoShootAction(robot_.getIntakeShooter(), robot_.getTargetTracker(), true, rotate_) ;        
        p1_ = new SwerveHolonomicPathFollower(robot.getSwerve(), "S2S2-P1", true, 0.2, mirror, mvalue);
        
        double v1 = robot_.getIntakeShooter().getUpDown().getSettingsValue("targets:stow").getDouble() ;
        double v2 = robot_.getIntakeShooter().getTilt().getSettingsValue("targets:stow").getDouble() ;         
        stow_ = new IntakeGotoNamedPositionAction(robot_.getIntakeShooter(), v1, v2) ;        
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        state_ = State.Shoot1 ;
        robot_.getIntakeShooter().setHoldingNote(true);
        robot_.getIntakeShooter().setAction(manual_shoot_, true) ;
    }

    private void idleState() {
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

    private void missedCollectState() {
        robot_.getIntakeShooter().setAction(stow_, true) ;        
        state_ = State.MissedStow ;
    }    

    private void missedStowState() {
        if (stow_.isDone()) {
            state_ = State.Done ;
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

    private void shoot2State() {
        if (shoot_.isDone()) {
            state_ = State.MissedCollect ;
        }
    }

    @Override
    public void run() throws Exception {
        super.run() ;

        switch(state_) {
        case Idle:
            idleState() ;
            break ;

        case Shoot1:
            shoot1State();
            break ;

        case Path1:
            path1State() ;
            break ;

        case Shoot2:
            shoot2State() ;
            break ;

        case FinishCollect1:
            finishCollect1State();
            break;

        case MissedCollect:
            missedCollectState();
            break ;

        case MissedStow:
            missedStowState();
            break ;

        case Done:
            break ;
        }
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "Start2Shoot2Action" ;
    }
}
