package frc.robot.subsystems.oi;

import org.xero1425.base.subsystems.motorsubsystem.MCMotionMagicAction;
import org.xero1425.base.subsystems.oi.OILed;
import org.xero1425.base.subsystems.oi.OIPanel;
import org.xero1425.base.subsystems.oi.OISubsystem;
import org.xero1425.base.subsystems.oi.OILed.State;
import org.xero1425.base.subsystems.oi.OIPanelButton.ButtonType;
import org.xero1425.base.subsystems.swerve.SwerveRotateToAngle;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

import frc.robot.subsystems.amp_trap.AmpTrapPositionAction;
import frc.robot.subsystems.amp_trap.AmpTrapShootAction;
import frc.robot.subsystems.intake_shooter.IntakeAutoShootAction;
import frc.robot.subsystems.intake_shooter.IntakeGotoNamedPositionAction;
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
        TransferToAmpTrap,
        NoteInAmpPositon,
        NoteInTrapPosition,
        NoteGoingToAmpPositon,
        NoteGoingToTrapPosition,
        ShootingAmp,
        StowingAmpTrap,
        GoToClimbPosition,
        WaitForClimbButton,
        Climbing,
        MovingTrapWhileUp,
        ShootingTrap,
        Climbed
    }

    //
    // Panel gadgets
    //
    private int target_toggle1_gadget_ ;                    // Speaker, Amp, or Trap
    private int target_toggle2_gadget_ ;                    // Speaker, Amp, or Trap     
    private int climb_up_prep_gadget_ ;                     // Prepare to climb up
    private int climb_up_exec_gadget_ ;                     // Perform the climb up
    private int shoot_gadget_ ;                             // Shoot the note
    private int abort_gadget_ ;                             // Abort mode

    private int climb_down_gadget_ ;                        // Prepare to climb down
    private int eject_gadget_ ;                             // Eject the note
    private int turtle_gadget_ ;                            // Turtle mode

    //
    // LEDs
    //
    private OILed db_ready_led_ ;
    private OILed shooter_velocity_ready_led_ ;
    private OILed shooter_tilt_ready_led_ ;
    private OILed shooter_april_tag_led_ ;

    private OILed climb_up_prep_enabled_led_ ;
    private OILed climb_up_exec_enabled_led_ ;
    private OILed climb_down_exec_enabled_led_ ;

    //
    // Actions
    //
    private IntakeAutoShootAction shoot_action_ ;
    private SwerveRotateToAngle rotate_action_ ;
    private TransferIntakeToTrampAction fwd_transfer_action_ ;
    private IntakeGotoNamedPositionAction stow_intake_action_ ;
    private AmpTrapPositionAction goto_amp_action_ ;
    private AmpTrapPositionAction goto_trap_action_ ;
    private AmpTrapPositionAction goto_trap_up_action_ ;
    private AmpTrapPositionAction stow_amp_trap_action_ ;
    private AmpTrapShootAction amp_trap_shoot_action_ ;
    private AmpTrapPositionAction goto_climb_pos_action_ ;
    private MCMotionMagicAction hooks_up_action_ ;
    private MCMotionMagicAction hooks_down_action_ ;    

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
        if (robot.getIntakeShooter().isHoldingNote()) {
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
            AllegroRobot2024 robot = (AllegroRobot2024)getSubsystem().getRobot().getRobotSubsystem() ;
            NoteTarget target = getNoteTarget() ;

            //
            // We have transferred the note to the manipulator on the elevator/arm.
            // Now, decide how to position the elevator/arm based on the target feature
            // (amp or trap).
            //
            if (target == NoteTarget.Amp) {
                state_ = OIState.NoteGoingToAmpPositon ;
                robot.getAmpTrap().setAction(goto_amp_action_, true) ;
            }
            else {
                state_ = OIState.NoteGoingToTrapPosition ;
                robot.getAmpTrap().setAction(goto_trap_action_, true) ;
                robot.getSuperStructure().getClimber().setAction(hooks_up_action_, true) ;
            }

            //
            // While we are doing this, also stow the intake shooter
            //
            robot.getIntakeShooter().setAction(stow_intake_action_, true) ;
        }
    }

    private void noteGoingToAmpPositonState() {
        if (goto_amp_action_.isDone()) {
            state_ = OIState.NoteInAmpPositon ;
        }
    }

    private void noteGoingToTrapPositionState() {
        if (goto_trap_action_.isDone()) {
            state_ = OIState.NoteInTrapPosition ;
        }
    }   

    private void noteInAmpPositonState() {
        if (getValue(shoot_gadget_) == 1) {
            AllegroRobot2024 robot = (AllegroRobot2024)getSubsystem().getRobot().getRobotSubsystem() ;
            robot.getAmpTrap().setAction(amp_trap_shoot_action_, true) ;
            state_ = OIState.ShootingAmp ;
        }
    }

    private void shootingAmpState() {
        if (amp_trap_shoot_action_.isDone()) {
            AllegroRobot2024 robot = (AllegroRobot2024)getSubsystem().getRobot().getRobotSubsystem() ;
            robot.getAmpTrap().setAction(stow_amp_trap_action_, true) ;            
            state_ = OIState.StowingAmpTrap ;
        }
    }

    private void stowingAmpTrapState() {
        if (stow_amp_trap_action_.isDone()) {
            state_ = OIState.Idle ;
        }
    }

    private void noteInTrapPositionState() {
        climb_up_prep_enabled_led_.setState(State.ON);
        if (getValue(climb_up_prep_gadget_) == 1) {
            AllegroRobot2024 robot = (AllegroRobot2024)getSubsystem().getRobot().getRobotSubsystem() ;            
            climb_up_prep_enabled_led_.setState(State.BLINK_FAST);
            robot.getAmpTrap().setAction(goto_climb_pos_action_, true) ;
            state_ = OIState.GoToClimbPosition ;
        }
    }

    private void gotoClimbPositionState() {
        if (goto_climb_pos_action_.isDone() && hooks_up_action_.isDone()) {
            climb_up_prep_enabled_led_.setState(State.OFF);
            climb_up_exec_enabled_led_.setState(State.ON);
            state_ = OIState.WaitForClimbButton ;
        }
    }

    private void waitForClimbButtonState() {
        if (hooks_up_action_.isDone() && getValue(climb_up_exec_gadget_) == 1) {
            climb_up_exec_enabled_led_.setState(State.BLINK_FAST);
            AllegroRobot2024 robot = (AllegroRobot2024)getSubsystem().getRobot().getRobotSubsystem() ;  
            robot.getSuperStructure().getClimber().setAction(hooks_down_action_, true) ;
            state_ = OIState.Climbing ;
        }
    }

    private void climbingState() {
        if (hooks_down_action_.isDone()) {
            climb_up_exec_enabled_led_.setState(State.OFF);

            AllegroRobot2024 robot = (AllegroRobot2024)getSubsystem().getRobot().getRobotSubsystem() ;  
            robot.getAmpTrap().setAction(goto_trap_up_action_, true) ;

            climb_down_exec_enabled_led_.setState(State.ON);
            state_ = OIState.MovingTrapWhileUp ;
        }
    }

    private void movingTrapWhileUpState() {
        if (goto_trap_up_action_.isDone()) {
            climb_down_exec_enabled_led_.setState(State.OFF);
            AllegroRobot2024 robot = (AllegroRobot2024)getSubsystem().getRobot().getRobotSubsystem() ;  
            robot.getAmpTrap().setAction(amp_trap_shoot_action_, true) ;
            state_ = OIState.ShootingTrap ;
        }
    }

    private void shootingTrapState() {
        if (amp_trap_shoot_action_.isDone()) {
            state_ = OIState.Climbed ;
            climb_down_exec_enabled_led_.setState(State.ON);
        }
    }

    private void climbedState() {
        if (getValue(climb_down_gadget_) == 1) {
            climb_down_exec_enabled_led_.setState(State.BLINK_FAST);

            //
            // TODO: execute the climb down
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

        case NoteGoingToAmpPositon:
            noteGoingToAmpPositonState() ;
            break ;

        case NoteGoingToTrapPosition:
            noteGoingToTrapPositionState() ;
            break ;

        case NoteInAmpPositon:
            noteInAmpPositonState();
            break ;

        case NoteInTrapPosition:
            noteInTrapPositionState();
            break ;

        case ShootingAmp:
            shootingAmpState() ;
            break ;

        case StowingAmpTrap:
            stowingAmpTrapState() ;
            break ;

        case GoToClimbPosition:
            gotoClimbPositionState() ;
            break ;

        case WaitForClimbButton:
            waitForClimbButtonState() ;
            break ;

        case Climbing:
            climbingState() ;
            break ;

        case MovingTrapWhileUp:
            movingTrapWhileUpState() ;
            break ;

        case ShootingTrap:
            shootingTrapState() ;
            break ;

        case Climbed:
            climbedState() ;
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

        double v1, v2 ;

        v1 = intake.getUpDown().getSettingsValue("target:stow").getDouble() ;
        v2 = intake.getTilt().getSettingsValue("target:stow").getDouble() ; 
        stow_intake_action_ = new IntakeGotoNamedPositionAction(intake, v1, v2) ;
        
        goto_amp_action_ = new AmpTrapPositionAction(robot.getAmpTrap(), "actions:amp:pivot", "actions:amp:elevator") ;
        goto_trap_action_ = new AmpTrapPositionAction(robot.getAmpTrap(), "actions:trap:pivot", "actions:trap:elevator") ;
        goto_climb_pos_action_ = new AmpTrapPositionAction(robot.getAmpTrap(), "actions:climb:pivot", "actions:climb:elevator") ;
        goto_trap_up_action_ = new AmpTrapPositionAction(robot.getAmpTrap(), "actions:trap-up:pivot", "actions:trap-up:elevator") ;

        amp_trap_shoot_action_ = new AmpTrapShootAction(robot.getAmpTrap()) ;

        stow_amp_trap_action_ = new AmpTrapPositionAction(robot.getAmpTrap(), "actions:stow:pivot", "actions:stow:elevator") ;

        hooks_up_action_ = new MCMotionMagicAction(robot.getSuperStructure().getClimber(), "pids:position", "targets:climb:up", 0.05, 0.05) ;
        hooks_down_action_ = new MCMotionMagicAction(robot.getSuperStructure().getClimber(), "pids:position", "targets:climb:down", 0.05, 0.05) ;
    }

    @Override
    protected void initializeLEDs() throws BadParameterTypeException, MissingParameterException {
        super.initializeLEDs();
        int num ;

        num = getSubsystem().getSettingsValue("panel:leds:db-ready").getInteger() ;
        db_ready_led_ = createLED(num, false) ;
        
        num = getSubsystem().getSettingsValue("panel:leds:shooter-velocity-ready").getInteger() ;
        shooter_velocity_ready_led_ = createLED(num, false) ;

        num = getSubsystem().getSettingsValue("panel:leds:shooter-tilt-ready").getInteger() ;
        shooter_tilt_ready_led_ = createLED(num, false) ;

        num = getSubsystem().getSettingsValue("panel:leds:shooter-april-tag").getInteger() ;
        shooter_april_tag_led_ = createLED(num, false) ;

        num = getSubsystem().getSettingsValue("panel:leds:climb-up-prep-enabled").getInteger() ;
        climb_up_prep_enabled_led_ = createLED(num, false) ;

        num = getSubsystem().getSettingsValue("panel:leds:climb-up-exec-enabled").getInteger() ;
        climb_up_exec_enabled_led_ = createLED(num, false) ;

        num = getSubsystem().getSettingsValue("panel:leds:climb-down-exec-enabled").getInteger() ;
        climb_down_exec_enabled_led_ = createLED(num, false) ;
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
