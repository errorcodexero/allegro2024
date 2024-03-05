package frc.robot.subsystems.superstructure;

import org.xero1425.base.actions.Action;

import frc.robot.subsystems.amp_trap.AmpTrapPositionAction;
import frc.robot.subsystems.intake_shooter.IntakeGotoNamedPositionAction;
import frc.robot.subsystems.superstructure.ClimbAction.ClimbType;

public class TurtleAction extends Action {
    private enum State {
        Idle,
        Climber,
        Elevator,
        Intake,
        Done
    } ;

    private SuperStructureSubsystem sub_;   
    private State state_ ;
    private ClimbAction climb_ ;
    
    private AmpTrapPositionAction elevator_ ;
    private IntakeGotoNamedPositionAction intake_ ;

    public TurtleAction(SuperStructureSubsystem sub) throws Exception {
        super(sub.getRobot().getMessageLogger());
        sub_ = sub;
        state_ = State.Idle;

        climb_ = new ClimbAction(sub_.getClimber(), ClimbType.HooksDown) ;
        elevator_ = new AmpTrapPositionAction(sub_.getAmpTrap(), "actions:stow:pivot", "actions:stow:elevator") ;

        double v1 = sub_.getIntakeShooter().getUpDown().getSettingsValue("targets:stow").getDouble() ;
        double v2 = sub_.getIntakeShooter().getTilt().getSettingsValue("targets:stow").getDouble() ; 
        intake_ = new IntakeGotoNamedPositionAction(sub_.getIntakeShooter(), v1, v2) ;
    }

    @Override
    public void start() throws Exception {
        super.start();
        sub_.getClimber().setAction(climb_, true);
        state_ = State.Climber;
    }

    @Override
    public void run() throws Exception {
        switch(state_) {
        case Idle:
            break;

        case Done:
            break ;

        case Climber:
            if (climb_.isDone()) {
                sub_.getAmpTrap().setAction(elevator_, true);
                state_ = State.Elevator ;
            }
            break ;

        case Elevator:
            if (elevator_.isDone()) {
                sub_.getIntakeShooter().setAction(intake_, true);
                state_ = State.Intake ;
            }
            break ;

        case Intake:
            if (intake_.isDone()) {
                setDone() ;
                state_ = State.Done ;
            }
            break ;
        }
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "TurtleAction" ;
    }
}
