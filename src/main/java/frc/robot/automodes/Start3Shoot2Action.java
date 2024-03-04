package frc.robot.automodes;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.swerve.SwerveHolonomicPathFollower;
import org.xero1425.base.subsystems.swerve.SwerveTrackAngle;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;

import frc.robot.subsystems.intake_shooter.IntakeAutoShootAction;
import frc.robot.subsystems.intake_shooter.IntakeGotoNamedPositionAction;
import frc.robot.subsystems.intake_shooter.StartCollectAltAction;
import frc.robot.subsystems.intake_shooter.StopCollectAltAction;
import frc.robot.subsystems.toplevel.AllegroRobot2024;

public class Start3Shoot2Action extends Action {
    private static final double kRotatePosTol = 2.0 ;

    private enum State {
        Idle,
        Shoot1,
        Path1,
        FinishCollect1,
        Path1Stowing,
        MissedCollect1,
        Shoot2,
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
    private StopCollectAltAction stop_collect_ ;
    private SwerveHolonomicPathFollower p1_ ;

    public Start3Shoot2Action(AllegroRobot2024 robot, boolean mirror, double mvalue) throws Exception {
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
        start_collect_ = new StartCollectAltAction(robot_.getIntakeShooter(), false) ;
        stop_collect_ = new StopCollectAltAction(robot_.getIntakeShooter()) ;

        p1_ = new SwerveHolonomicPathFollower(robot.getSwerve(), "S3S3-P1", true, 0.2, mirror_, mvalue_);
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
            robot_.getSwerve().enableVision(false);
            robot_.getIntakeShooter().setAction(stow_, true) ;
            robot_.getSwerve().setAction(p1_, true) ;
            state_ = State.Path1Stowing ;
        }
    }

    private void path1StowingState() {
        if (stow_.isDone()) {
            robot_.getIntakeShooter().setAction(start_collect_, true) ;
            state_ = State.Path1 ;
        }
    }

    private void path1State() {
        if (p1_.isDone() && robot_.getIntakeShooter().isHoldingNote()) {
            state_ = State.FinishCollect1 ;
        }
        else if (p1_.isDone()) {
            state_ = State.MissedCollect1 ;
        }
    }

    private void finishCollect1State() {
        if (start_collect_.isDone()) {
            robot_.getSwerve().setAction(rotate_, true) ;
            robot_.getIntakeShooter().setAction(shoot_, true) ;
            state_ = State.Shoot2 ;
        }
    }

    private void missedCollect1State() {
        if (stop_collect_.isDone()) {
            //
            // We just stop here for now (maybe do more later)
            //
            state_ = State.Done ;
        }
    }

    private void shoot2State() {
        if (shoot_.isDone()) {
            state_ = State.Done ;
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

            case Path1Stowing:
                path1StowingState() ;
                break ;

            case MissedCollect1:
                missedCollect1State() ;
                break ;

            case Shoot2:
                shoot2State() ;
                break ;

            case FinishCollect1:
                finishCollect1State() ;
                break ;
        }

        if (prev != state_) {
            MessageLogger logger = robot_.getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Info);
            logger.add("Start2Shoot4Action action changed states: " + prev.toString() + " -> " + state_.toString()) ;
            logger.endMessage();
        }
    }

    @Override
    public String toString(int indent) {
        return prefix(indent) + "Start2Shoot4Action" ;
    }
}
