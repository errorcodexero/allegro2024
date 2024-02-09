package frc.robot.subsystems.oi;

import org.xero1425.base.subsystems.oi.OILed;
import org.xero1425.base.subsystems.oi.OIPanel;
import org.xero1425.base.subsystems.oi.OISubsystem;
import org.xero1425.base.subsystems.oi.OIPanelButton.ButtonType;
import org.xero1425.base.subsystems.swerve.SwerveRotateToAngle;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

import frc.robot.subsystems.intake_shooter.IntakeAutoShootAction;
import frc.robot.subsystems.intake_shooter.IntakeShooterSubsystem;
import frc.robot.subsystems.superstructure.TransferIntakeToTrampAction;
import frc.robot.subsystems.target_tracker.TargetTrackerSubsystem;
import frc.robot.subsystems.toplevel.AllegroRobot2024;

public class ButchOIPanel extends OIPanel {
    enum NoteTarget {
        Invalid,
        Trap,
        Amp,
        Speaker
    } ;

    enum OIState {
        Idle,
        NoteInIntake,
        WaitingToShoot,
        Shooting,
        TransferToAmpTrap
    }

    //
    // Panel gadgets
    //
    private int target_toggle1_gadget_ ;                    // Speaker, Amp, or Trap
    private int target_toggle2_gadget_ ;                    // Speaker, Amp, or Trap    
    private int amp_purpose_target1_gadget_ ;               // Amplification, Cooperition, None
    private int amp_purpose_target2_gadget_ ;               // Amplification, Cooperition, None    
    private int climb_up_prep_gadget_ ;                     // Prepare to climb up
    private int climb_up_exec_gadget_ ;                     // Perform the climb up
    private int climb_down_gadget_ ;                        // Prepare to climb down
    private int shoot_gadget_ ;                             // Shoot the note
    private int turtle_gadget_ ;                            // Turtle mode
    private int abort_gadget_ ;                             // Abort mode
    private int eject_gadget_ ;                             // Eject the note

    //
    // LEDs
    //
    private OILed db_ready_led_ ;
    private OILed shooter_velocity_ready_led_ ;
    private OILed shooter_tilt_ready_led_ ;
    private OILed shooter_april_tag_led_ ;
    private OILed note_in_manipulator_led_ ;
    private OILed climb_up_prep_enabled_led_ ;
    private OILed climb_up_exec_enabled_led_ ;
    private OILed climb_down_prep_enabled_led_ ;
    private OILed climb_down_exec_enabled_led_ ;

    //
    // Actions
    //
    private IntakeAutoShootAction shoot_action_ ;
    private SwerveRotateToAngle rotate_action_ ;
    private TransferIntakeToTrampAction fwd_transfer_action_ ;

    //
    // Misc
    //
    private OIState state_ ;

    public ButchOIPanel(OISubsystem oi, int index) throws BadParameterTypeException, MissingParameterException {
        super(oi, "oipanel", index);

        state_ = OIState.Idle ;
    }

    @Override
    public void computeState() throws Exception {
        super.computeState();
    }

    private void idleState() {
        AllegroRobot2024 robot = (AllegroRobot2024)getSubsystem().getRobot().getRobotSubsystem() ;
        if (robot.getIntakeShooter().isNoteInIntake()) {
            state_ = OIState.NoteInIntake ;
        }
    }

    private void noteInIntakeState() {
        AllegroRobot2024 robot = (AllegroRobot2024)getSubsystem().getRobot().getRobotSubsystem() ;       
        NoteTarget target = getNoteTarget() ;

        if (target == NoteTarget.Speaker) {
            robot.getIntakeShooter().setAction(shoot_action_);            
            state_ = OIState.WaitingToShoot ;
        }
        else if (target == NoteTarget.Amp || target == NoteTarget.Trap) {
            robot.getSuperStructure().setAction(fwd_transfer_action_) ;
            state_ = OIState.TransferToAmpTrap;
        }
    }

    private void waitingToShootState() {
        AllegroRobot2024 robot = (AllegroRobot2024)getSubsystem().getRobot().getRobotSubsystem() ;

        NoteTarget target = getNoteTarget() ;
        if (target != NoteTarget.Speaker) {
            //
            // We have switched to a different target, abort the shoot action
            //
            shoot_action_.cancel() ;
            state_ = OIState.NoteInIntake ;
        }
        else {
            //
            // We are waiting to shoot.  The user must press the shoot button
            // to actually complete the shoot operation.
            //
            if (getValue(shoot_gadget_) == 1) {
                //
                // Find the desired absolute angle for the robot to aim at the goal.
                //
                double angle = robot.getTargetTracker().getRotation() + robot.getSwerve().getPose().getRotation().getDegrees() ;
                rotate_action_.setAngle(angle) ;

                //
                // Tell the drive base to rotate to the desired angle
                //
                robot.getSwerve().setAction(rotate_action_) ;
                state_ = OIState.Shooting ;

                //
                // Tell the shoot action that the drive team hit the shoot button
                //
                shoot_action_.setDriveTeamReady(true);
            }
        }
    }

    private void shootingState() {
        if (getValue(abort_gadget_) == 1) {
            shoot_action_.cancel() ;
            state_ = OIState.NoteInIntake ;
        }
        else {
            if (rotate_action_.isDone()) {
                shoot_action_.setDBReady(true);
            }

            if (shoot_action_.isDone()) {
                state_ = OIState.Idle ;
            }
        }
    }

    private void transferToAmpTrapState() {
        if (fwd_transfer_action_.isDone()) {
            //
            // TODO: after we see the actual hardware
            //
        }
    }

    @Override
    public void generateActions() {
        super.generateActions();

        switch(state_) {
        case Idle:
            idleState() ;
            break ;

        case NoteInIntake:
            noteInIntakeState() ;
            break ;

        case WaitingToShoot:
            waitingToShootState() ;
            break;

        case Shooting:
            shootingState() ;
            break ;

        case TransferToAmpTrap:
            transferToAmpTrapState() ;
            break ;
        }
    }

    @Override
    public void createStaticActions() throws Exception {
        AllegroRobot2024 robot = (AllegroRobot2024)getSubsystem().getRobot().getRobotSubsystem() ;
        IntakeShooterSubsystem intake = robot.getIntakeShooter() ;
        TargetTrackerSubsystem tracker = robot.getTargetTracker() ;

        shoot_action_ = new IntakeAutoShootAction(intake, tracker) ;
        rotate_action_ = new SwerveRotateToAngle(robot.getSwerve(), 0.0, 5.0, 1.0) ;
        fwd_transfer_action_ = new TransferIntakeToTrampAction(robot.getSuperStructure()) ;
    }    

    @Override
    protected void initializeLEDs() throws BadParameterTypeException, MissingParameterException {
        super.initializeLEDs();
        int num ;

        num = getSubsystem().getSettingsValue("panel:leds:db-ready").getInteger() ;
        db_ready_led_ = createLED(num) ;
        
        num = getSubsystem().getSettingsValue("panel:leds:shooter-velocity-ready").getInteger() ;
        shooter_velocity_ready_led_ = createLED(num) ;

        num = getSubsystem().getSettingsValue("panel:leds:shooter-tilt-ready").getInteger() ;
        shooter_tilt_ready_led_ = createLED(num) ;

        num = getSubsystem().getSettingsValue("panel:leds:shooter-april-tag").getInteger() ;
        shooter_april_tag_led_ = createLED(num) ;

        num = getSubsystem().getSettingsValue("panel:leds:note-in-manipulator").getInteger() ;
        note_in_manipulator_led_ = createLED(num) ;

        num = getSubsystem().getSettingsValue("panel:leds:climb-up-prep-enabled").getInteger() ;
        climb_up_prep_enabled_led_ = createLED(num) ;

        num = getSubsystem().getSettingsValue("panel:leds:climb-up-exec-enabled").getInteger() ;
        climb_up_exec_enabled_led_ = createLED(num) ;

        num = getSubsystem().getSettingsValue("panel:leds:climb-down-prep-enabled").getInteger() ;
        climb_down_prep_enabled_led_ = createLED(num) ;

        num = getSubsystem().getSettingsValue("panel:leds:climb-down-exec-enabled").getInteger() ;
        climb_down_exec_enabled_led_ = createLED(num) ;
    }

    @Override
    protected void initializeGadgets() throws BadParameterTypeException, MissingParameterException {
        super.initializeGadgets();
        int num ;

        num = getSubsystem().getSettingsValue("panel:gadgets:target_toggle1").getInteger() ;
        target_toggle1_gadget_ = mapButton(num, ButtonType.Level) ;

        num = getSubsystem().getSettingsValue("panel:gadgets:target_toggle2").getInteger() ;
        target_toggle2_gadget_ = mapButton(num, ButtonType.Level) ;

        num = getSubsystem().getSettingsValue("panel:gadgets:climb_up_prep").getInteger() ;
        climb_up_prep_gadget_ = mapButton(num, ButtonType.LowToHigh) ;

        num = getSubsystem().getSettingsValue("panel:gadgets:climb_up_exec").getInteger() ;
        climb_up_exec_gadget_ = mapButton(num, ButtonType.LowToHigh) ;

        num = getSubsystem().getSettingsValue("panel:gadgets:climb_down").getInteger() ;
        climb_down_gadget_ = mapButton(num, ButtonType.LowToHigh) ;

        num = getSubsystem().getSettingsValue("panel:gadgets:shoot").getInteger() ;
        shoot_gadget_ = mapButton(num, ButtonType.LowToHigh) ;

        num = getSubsystem().getSettingsValue("panel:gadgets:shoot").getInteger() ;
        turtle_gadget_ = mapButton(num, ButtonType.LowToHigh) ;
        
        num = getSubsystem().getSettingsValue("panel:gadgets:shoot").getInteger() ;
        abort_gadget_ = mapButton(num, ButtonType.LowToHigh) ;
        
        num = getSubsystem().getSettingsValue("panel:gadgets:shoot").getInteger() ;
        eject_gadget_ = mapButton(num, ButtonType.LowToHigh) ;
    }

    private NoteTarget getNoteTarget() {
        NoteTarget ret = NoteTarget.Invalid ;

        if (getValue(target_toggle1_gadget_) == 1 && getValue(target_toggle2_gadget_) == 0) {
            ret = NoteTarget.Speaker ;
        }
        else if (getValue(target_toggle1_gadget_) == 0 && getValue(target_toggle2_gadget_) == 0) {
            ret = NoteTarget.Amp ;
        }
        else if (getValue(target_toggle1_gadget_) == 0 && getValue(target_toggle2_gadget_) == 1) {
            ret = NoteTarget.Trap ;
        }

        return ret ;
    }
}
