package frc.robot.subsystems.superstructure;

import org.xero1425.base.actions.Action;

import frc.robot.subsystems.amp_trap.AmpTrapEjectAction;
import frc.robot.subsystems.intake_shooter.IntakeEjectAction;
import frc.robot.subsystems.intake_shooter.IntakeGotoNamedPositionAction;

public class EjectAction extends Action {
    
    private enum State {
        Idle,
        ToTee,
        RunStuff,
        Done
    }

    private SuperStructureSubsystem sub_;
    private State state_;

    private IntakeGotoNamedPositionAction to_tee_;
    private IntakeEjectAction intake_eject_ ;
    private AmpTrapEjectAction amp_eject_ ;

    public EjectAction(SuperStructureSubsystem sub) throws Exception {
        super(sub.getRobot());
        sub_ = sub;
        state_ = State.Idle;

        double v1 = sub_.getIntakeShooter().getUpDown().getSettingsValue("targets:stow").getDouble() ;
        double v2 = sub_.getIntakeShooter().getTilt().getSettingsValue("targets:stow").getDouble() ;         
        to_tee_ = new IntakeGotoNamedPositionAction(sub_.getIntakeShooter(), v1, v2) ;

        intake_eject_ = new IntakeEjectAction(sub_.getIntakeShooter()) ;
        amp_eject_ = new AmpTrapEjectAction(sub_.getAmpTrap()) ;
    }

    @Override
    public void start() throws Exception {
        super.start();
        sub_.getIntakeShooter().setAction(to_tee_, true);
        state_ = State.ToTee;
    }

    @Override
    public void run() throws Exception {
        switch(state_) {
        case Idle:
            break;
            
        case Done:
            break ;

        case ToTee:
            if (to_tee_.isDone()) {
                sub_.getIntakeShooter().setAction(intake_eject_, true);
                sub_.getAmpTrap().setAction(amp_eject_, true);
                state_ = State.RunStuff;
            }
            break;

        case RunStuff:
            if (intake_eject_.isDone() && amp_eject_.isDone()) {
                state_ = State.Done;
                setDone();
            }
            break;
        }
    }

    @Override
    public String toString(int indent) {
        return prefix(indent) + "EjectAction" ;
    }
}
