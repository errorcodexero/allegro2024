package frc.robot.subsystems.superstructure;

import org.xero1425.base.actions.Action;

import frc.robot.subsystems.amp_trap.AmpTrapPositionAction;
import frc.robot.subsystems.amp_trap.AmpTrapXferAction;
import frc.robot.subsystems.intake_shooter.IntakeGotoNamedPositionAction;
import frc.robot.subsystems.intake_shooter.IntakeShooterXferAction;

public class TransferIntakeToTrampAction extends Action {
    private SuperStructureSubsystem sub_ ;

    private AmpTrapPositionAction amp_trap_position_action_ ;
    private IntakeGotoNamedPositionAction intake_shooter_position_action_ ;
    private AmpTrapXferAction amp_trap_xfer_action_ ;
    private IntakeShooterXferAction intake_shooter_xfer_action_ ;

    private boolean doing_xfer_ ;
    private double xfer_length_ ;
    private double start_pos_ ;

    public TransferIntakeToTrampAction(SuperStructureSubsystem sub) throws Exception {
        super(sub.getRobot().getMessageLogger());
        
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
    }

    @Override
    public void start() throws Exception {
        super.start();

        start_pos_ = sub_.getIntakeShooter().getShooter1().getPosition() ;
        doing_xfer_ = false ;
        sub_.getAmpTrap().setAction(amp_trap_position_action_, true);
        sub_.getIntakeShooter().setAction(intake_shooter_position_action_, true);
    }

    @Override
    public void run() throws Exception {
        if(doing_xfer_) {
            if (sub_.getIntakeShooter().getShooter1().getPosition() - start_pos_ > xfer_length_) {
                sub_.getAmpTrap().getManipulator().setPower(0.0);
                sub_.getIntakeShooter().getFeeder().setPower(0.0);
                sub_.getIntakeShooter().getShooter1().setPower(0.0);
                sub_.getIntakeShooter().getShooter2().setPower(0.0);

                sub_.getIntakeShooter().setHoldingNote(false);
                sub_.getAmpTrap().setHoldingNote(true);
                setDone();
            }
        }
        else {
            if (amp_trap_position_action_.isDone() && intake_shooter_position_action_.isDone()) {
                doing_xfer_ = true ;
                start_pos_ = sub_.getIntakeShooter().getFeeder().getPosition() ;

                sub_.getAmpTrap().setAction(amp_trap_xfer_action_, true);
                sub_.getIntakeShooter().setAction(intake_shooter_xfer_action_, true);
            }
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
