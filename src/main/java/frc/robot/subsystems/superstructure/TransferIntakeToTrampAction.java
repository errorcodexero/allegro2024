package frc.robot.subsystems.superstructure;

import org.xero1425.base.actions.Action;
import org.xero1425.misc.MessageType;

import frc.robot.subsystems.amp_trap.AmpTrapPositionAction;
import frc.robot.subsystems.amp_trap.AmpTrapXferAction;
import frc.robot.subsystems.intake_shooter.IntakeGotoNamedPositionAction;
import frc.robot.subsystems.intake_shooter.IntakeShooterXferAction;

public class TransferIntakeToTrampAction extends Action {
    private enum State {
        MovingToPosition,
        WaitingOnSensorNone1,
        WaitingOnSensorDetected,
        WaitingOnSensorNone2,
        FinishingTransfer,
        ContinueShooter,
    } ;

    private SuperStructureSubsystem sub_ ;

    private AmpTrapPositionAction amp_trap_position_action_ ;
    private IntakeGotoNamedPositionAction intake_shooter_position_action_ ;
    private AmpTrapXferAction amp_trap_xfer_action_ ;
    private IntakeShooterXferAction intake_shooter_xfer_action_ ;

    private double xfer_length_ ;
    private double cont_length_ ;
    private double start_pos_ ;

    private State state_ ;

    public TransferIntakeToTrampAction(SuperStructureSubsystem sub) throws Exception {
        super(sub.getRobot());
        
        double v1, v2 ;

        sub_ = sub ;

        v1 = sub_.getSettingsValue("actions:xfer:arm").getDouble() ;
        v2 = sub_.getSettingsValue("actions:xfer:elevator").getDouble() ;

        amp_trap_position_action_ = new AmpTrapPositionAction(sub_.getAmpTrap(), v1, v2) ;

        v1 = sub_.getSettingsValue("actions:xfer:updown").getDouble() ;
        v2 = sub_.getSettingsValue("actions:xfer:tilt").getDouble() ;
        intake_shooter_position_action_ = new IntakeGotoNamedPositionAction(sub_.getIntakeShooter(), v1, v2) ;
      
        amp_trap_xfer_action_ = new AmpTrapXferAction(sub_.getAmpTrap()) ;

        v1 = sub.getSettingsValue("actions:xfer:feeder-power").getDouble() ;
        v2 = sub.getSettingsValue("actions:xfer:shooter-velocity").getDouble() ;
        intake_shooter_xfer_action_ = new IntakeShooterXferAction(sub_.getIntakeShooter(), v1, v2) ;

        xfer_length_ = sub_.getSettingsValue("actions:xfer:shooter-length").getDouble() ;
        cont_length_ = sub_.getSettingsValue("actions:xfer:cont-length").getDouble() ;
    }

    @Override
    public void start() throws Exception {
        super.start();

        state_ = State.MovingToPosition ;
        sub_.getAmpTrap().getManipulator().cancelAction();
        sub_.getAmpTrap().getManipulator().setPower(0.0);
        start_pos_ = sub_.getIntakeShooter().getShooter1().getPosition() ;
        sub_.getAmpTrap().setAction(amp_trap_position_action_, true);
        sub_.getIntakeShooter().setAction(intake_shooter_position_action_, true);
    }

    @Override
    public void run() throws Exception {
        super.run() ;

        State prev = state_ ;
        
        switch(state_) {
        case MovingToPosition:
            if (amp_trap_position_action_.isDone() && intake_shooter_position_action_.isDone()) {
                start_pos_ = sub_.getIntakeShooter().getShooter1().getPosition() ;                
                sub_.getAmpTrap().setAction(amp_trap_xfer_action_, true);
                sub_.getIntakeShooter().setAction(intake_shooter_xfer_action_, true);

                state_ = State.WaitingOnSensorNone1 ;
            }
            break ;

        case WaitingOnSensorNone1:
            if (!sub_.getIntakeShooter().isNoteCurrentlyDetected()) {
                state_ = State.WaitingOnSensorDetected ;
            }
            break ;

        case WaitingOnSensorDetected:
            if (sub_.getIntakeShooter().isNoteCurrentlyDetected()) {
                state_  = State.WaitingOnSensorNone2 ;
            }
            break ;

        case WaitingOnSensorNone2:
            if (!sub_.getIntakeShooter().isNoteCurrentlyDetected()) {
                state_ = State.FinishingTransfer ;
                start_pos_ = sub_.getIntakeShooter().getShooter1().getPosition() ;
            }
            break ;

        case FinishingTransfer:
            if (sub_.getIntakeShooter().getShooter1().getPosition() - start_pos_ > xfer_length_) {
                amp_trap_xfer_action_.cancel() ;
                sub_.getIntakeShooter().setHoldingNote(false);
                sub_.getAmpTrap().setHoldingNote(true);
                state_ = State.ContinueShooter ;
            }        
            break; 

        case ContinueShooter:
            if (sub_.getIntakeShooter().getShooter1().getPosition() - start_pos_ - xfer_length_ > cont_length_) {
                intake_shooter_xfer_action_.cancel();
                setDone();
            }
            break ;
        }

        if (state_ != prev) {
            sub_.getRobot().getMessageLogger().startMessage(MessageType.Debug, sub_.getLoggerID()) ;
            sub_.getRobot().getMessageLogger().add("transfer note state changed: ");
            sub_.getRobot().getMessageLogger().add(prev.toString()) ;
            sub_.getRobot().getMessageLogger().add(" ->");            
            sub_.getRobot().getMessageLogger().add(state_.toString()) ;            
            sub_.getRobot().getMessageLogger().endMessage();            
        }
    }

    @Override
    public void cancel() {
    }

    @Override
    public String toString(int indent) {
        return prefix(indent) + "TransferIntakeToTrampAction";
    }
}
