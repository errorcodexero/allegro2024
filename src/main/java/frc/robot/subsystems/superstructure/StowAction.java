package frc.robot.subsystems.superstructure;

import org.xero1425.base.actions.Action;

import frc.robot.subsystems.amp_trap.AmpTrapPositionAction;
import frc.robot.subsystems.intake_shooter.IntakeGotoNamedPositionAction;

public class StowAction extends Action {
    private enum State {
        Idle,
        WaitingOnAmpTrapSafe,
        WaitingHooksDown,
        WaitingOnStows,
    } ;

    private SuperStructureSubsystem sub_ ;

    private ClimbAction hooks_down_ ;
    private AmpTrapPositionAction goto_safe_pos_for_climber_ ;
    private IntakeGotoNamedPositionAction intake_stow_ ;
    private AmpTrapPositionAction amp_trap_stow_ ;

    private double climber_down_pos_ ;

    private State state_ ;

    public StowAction(SuperStructureSubsystem sub) throws Exception {
        super(sub.getRobot().getMessageLogger());

        sub_ = sub ;

        climber_down_pos_ = sub_.getClimber().getSettingsValue("actions:climb:hooks-down:target").getDouble() ;
        goto_safe_pos_for_climber_ = new AmpTrapPositionAction(sub_.getAmpTrap(), "actions:climb:pivot", "actions:climb:elevator") ;
        amp_trap_stow_ = new AmpTrapPositionAction(sub_.getAmpTrap(), "actions:stow:pivot", "actions:stow:elevator") ;
        intake_stow_ = new IntakeGotoNamedPositionAction(sub_.getIntakeShooter(), "actions:stow:updown", "actions:stow:tilt") ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        if (sub_.getClimber().getPosition() > climber_down_pos_) {
            //
            // Need to move the elevator to a position that is safe for the climber
            //
            sub_.getAmpTrap().setAction(goto_safe_pos_for_climber_, true) ;
            state_ = State.WaitingOnAmpTrapSafe ;
        }
        else {
            sub_.getAmpTrap().setAction(amp_trap_stow_, true) ;
            sub_.getIntakeShooter().setAction(intake_stow_, true) ;
            state_ = State.WaitingOnStows ;
        }
    }

    @Override
    public void run() throws Exception {
        super.run() ;

        switch(state_) {
            case Idle:
                break ;

            case WaitingOnAmpTrapSafe:
                if (goto_safe_pos_for_climber_.isDone()) {
                    sub_.getClimber().setAction(hooks_down_) ;
                    state_ = State.WaitingHooksDown ;
                }

            case WaitingHooksDown:
                if (hooks_down_.isDone()) {
                    sub_.getAmpTrap().setAction(amp_trap_stow_, true) ;
                    sub_.getIntakeShooter().setAction(intake_stow_, true) ;
                    state_ = State.WaitingOnStows ;                    
                }
                break ;

            case WaitingOnStows:
                if (amp_trap_stow_.isDone() && intake_stow_.isDone()) {
                    setDone() ;
                }
                break ;
        }        
    }

    @Override
    public String toString(int indet) {
        return spaces(indet) + "StowAction" ;
    }
}
