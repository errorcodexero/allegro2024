package frc.robot.subsystems.oi;

import org.xero1425.base.subsystems.DriveBaseSubsystem;
import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.base.subsystems.oi.Gamepad;
import org.xero1425.base.subsystems.oi.OILed;
import org.xero1425.base.subsystems.oi.OILed.LEDState;
import org.xero1425.base.subsystems.oi.OISubsystem;
import org.xero1425.base.subsystems.oi.SwerveDriveGamepad;
import org.xero1425.base.subsystems.swerve.SwerveTrackAngle;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;

import frc.robot.subsystems.amp_trap.AmpTrapEjectAction;
import frc.robot.subsystems.amp_trap.AmpTrapMoveNote;
import frc.robot.subsystems.amp_trap.AmpTrapPositionAction;
import frc.robot.subsystems.intake_shooter.IntakeAutoShootAction;
import frc.robot.subsystems.intake_shooter.IntakeEjectAction;
import frc.robot.subsystems.intake_shooter.IntakeGotoNamedPositionAction;
import frc.robot.subsystems.intake_shooter.IntakeShooterSubsystem;
import frc.robot.subsystems.intake_shooter.StartCollectAltAction;
import frc.robot.subsystems.intake_shooter.StopCollectAltAction;
import frc.robot.subsystems.oi.AllegroOIPanel.NoteTarget;
import frc.robot.subsystems.superstructure.ClimbAction;
import frc.robot.subsystems.superstructure.TransferIntakeToTrampAction;
import frc.robot.subsystems.target_tracker.TargetTrackerSubsystem;
import frc.robot.subsystems.toplevel.AllegroRobot2024;

public class Allegro2024OISubsystem extends OISubsystem {
    private final static String OIHIDIndexName = "panel:index" ;

    enum OIState {
        Idle,
        Collect,
        EndingCollect,
        WaitingToShoot,
        Shooting,
        StowIntake,
        TransferToAmpTrap,
        NoteInAmpPosition,
        NoteInTrapPosition,
        NoteGoingToAmpPositon,
        MoveNoteToBack,
        NoteGoingToTrapPosition,
        ShootingAmp,
        StowingAmpTrap,
        GoToClimbPosition,
        WaitForHooksUp,
        WaitForHooksDown,
        WaitForClimbButton,
        WaitForHooksReset,
        ClimbingUp,
        MovingTrapWhileUp,
        ShootingTrap,
        Climbed,
        PrepToClimbDown,
        ClimbingDown,
        Eject
    }    

    private AllegroOIPanel oipanel_ ;

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    // Actions
    //
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //
    // This action is used to start the collect sequence
    //
    private StartCollectAltAction startCollectAction_ ;

    //
    // This action is used to stop the collect sequence
    //
    private StopCollectAltAction stopCollectAction_ ;

    //
    // This puts the intake-shooter into shoot mode.  The tilt and shooter wheels will start moving to align to the target
    // as the robot moves.  When the robot is in the correct position, the drive team must press the shoot button to actually
    // shoot the note.
    //
    private IntakeAutoShootAction shoot_action_ ;

    //
    // This action aligns the drivebase to the target.  It is used when the drive team presses the shoot button to actually
    // shoot the note.
    //
    private SwerveTrackAngle rotate_action_ ;

    //
    // THis action transfer a note from the intake-shooter to the manipulator on the elevator/arm
    //
    private TransferIntakeToTrampAction fwd_transfer_action_ ;

    //
    // This action stows the intake-shooter into it default stowed (shooting) position
    //
    private IntakeGotoNamedPositionAction stow_intake_action_ ;

    //
    // This action moves the note into the trap
    //
    private AmpTrapMoveNote trap_shoot_action_ ;

    //
    // This action moves the note into the amp
    //
    private AmpTrapMoveNote amp_shoot_action_ ;

    //
    // This action moves the climber hooks up to the height to climb onto the chain.
    //
    private ClimbAction hooks_up_action_ ;

    //
    // This actions moves the hooks down without the robot when a climb is aborted
    //
    private ClimbAction hooks_down_ ;

    //
    // This action moves the climber hooks down to cause the robot to climb.
    private ClimbAction hooks_down_with_robot_action_ ;

    //
    // The eject action for the intake
    //
    private IntakeEjectAction intake_eject_action_ ;

    //
    // The eject action for the arm/trap
    //
    private AmpTrapEjectAction trap_eject_action_ ;

    //
    // This action moves the elevator/arm to the position to shoot the note into the amp
    //
    private AmpTrapPositionAction amp_prep_pos_action_ ;

    //
    // This action moves the elevator/arm to the position to climb up to the trap
    //
    private AmpTrapPositionAction goto_trap_place_pos_action_ ;

    //
    // After climbing, this action moves the elevator/arm to the position to shoot the note into the trap
    //
    private AmpTrapPositionAction goto_trap_up_action_ ;

    //
    // This action stows the elevator/arm into the default stowed position after
    //
    private AmpTrapPositionAction stow_amp_trap_action_ ;

    //
    // This action moves the elevator/arm to the position to climb up to the trap
    //
    private AmpTrapPositionAction goto_climb_pos_action_ ;

    //
    // This action moves the elevator/arm to the position to climb down from the trap
    //
    private AmpTrapPositionAction goto_climb_down_pos_action_ ;

    //
    // Move the note to the proper position for the trap placement
    //
    private AmpTrapMoveNote move_note_action_ ;

    //
    // If true, we are doing a climb and not a trap placement
    //
    private boolean climb_only_ ;

    //
    // Current OI state
    //
    private OIState state_ ;    

    public Allegro2024OISubsystem(Subsystem parent, DriveBaseSubsystem db, IntakeShooterSubsystem intake) throws Exception {
        super (parent,"allegro2024oi",GamePadType.Swerve, db, true);

        int index ;
        MessageLogger logger = getRobot().getMessageLogger() ;

        try {
            index = getSettingsValue(OIHIDIndexName).getInteger() ;
        } catch (BadParameterTypeException e) {
            logger.startMessage(MessageType.Error) ;
            logger.add("parameter ").addQuoted(OIHIDIndexName) ;
            logger.add(" exists, but is not an integer").endMessage();
            index = -1 ;
        } catch (MissingParameterException e) {
            logger.startMessage(MessageType.Error) ;
            logger.add("parameter ").addQuoted(OIHIDIndexName) ;
            logger.add(" does not exist").endMessage();
            index = -1 ;            
        }

        if (index != -1) {
            try {
                oipanel_ = new AllegroOIPanel(this, index) ;
                addHIDDevice(oipanel_) ;
            }
            catch(Exception ex) {
                logger.startMessage(MessageType.Error) ;
                logger.add("OI HID device was not created - ") ;
                logger.add(ex.getMessage()).endMessage(); ;
            }
        }

        climb_only_ = false ;
    }

    public AllegroOIPanel getPanel() {
        return oipanel_ ;
    }

    @Override
    public boolean isCoastMode() {
        return oipanel_.getCoastValue() == 0 ;
    }

    public void targetLockMode(boolean enable) {
        AllegroRobot2024 robot = (AllegroRobot2024)getRobot().getRobotSubsystem();
        TargetTrackerSubsystem tracker = robot.getTargetTracker() ;
        SwerveDriveGamepad gp = (SwerveDriveGamepad)getGamePad() ;

        if (enable) {
            gp.setTrackingSupplier(() -> tracker.getRotation());
        }
        else {
            gp.setTrackingSupplier(null);
        }
    }

    @Override
    public void computeMyState() throws Exception {
        super.computeMyState();
    }

    public void scaleDriveBaseVelocities(double scale) {
        SwerveDriveGamepad gp = (SwerveDriveGamepad)getGamePad();
        gp.setScaleAmount(scale) ;
    }

    @Override
    protected void gamePadCreated(Gamepad gp) {
        SwerveDriveGamepad swgp = (SwerveDriveGamepad)gp ;
        if (swgp != null) {
            //
            // These are all bound directoy to the gamepad as they deal with the drivebase only
            // Any other buttons that are used to control shooting or climbing are queried in the
            // state machine below.
            //
            swgp.bindButtons(new Gamepad.Button[] {Gamepad.Button.Y, Gamepad.Button.B }, ()->swgp.resetSwerveDriveDirection(), null);
            swgp.bindButton(Gamepad.Button.LBack, ()-> swgp.startDriveBaseX(), ()->swgp.stopDriveBaseX());   

            swgp.bindButton(Gamepad.Button.LTrigger, ()->targetLockMode(true), null);
            swgp.bindButton(Gamepad.Button.RTrigger, ()->targetLockMode(false), null);

            // swgp.bindButton(Gamepad.Button.LTrigger, ()->scaleDriveBaseVelocities(0.25), ()->scaleDriveBaseVelocities(1.0));
        }
    }

    private void dispositionNoteInIntake() {
        AllegroRobot2024 robot = (AllegroRobot2024)getRobot().getRobotSubsystem() ;
        robot.getOI().getGamePad().rumble(1.0, 1.0) ;
        NoteTarget target = oipanel_.getNoteTarget() ;         
        if (target == NoteTarget.Speaker) {
            robot.getIntakeShooter().setAction(shoot_action_, true);         
            state_ = OIState.WaitingToShoot ;
        }
        else if (target == NoteTarget.Amp || target == NoteTarget.Trap) {
            robot.getSuperStructure().setAction(fwd_transfer_action_) ;
            state_ = OIState.TransferToAmpTrap;
        }          
    }

    private void idleState() {
        AllegroRobot2024 robot = (AllegroRobot2024)getRobot().getRobotSubsystem() ;
        SwerveDriveGamepad gp = (SwerveDriveGamepad)getGamePad() ;

        if (gp.isRBackButtonPressed()) {
            robot.getSuperStructure().setAction(startCollectAction_) ;
            state_ = OIState.Collect ;
        }
        else if (oipanel_.isClimbUpPrepPressed()) {
            climb_only_ = true ;
            robot.getAmpTrap().setAction(goto_climb_pos_action_, true) ;
            state_ = OIState.GoToClimbPosition ;
        }
    }

    private void collectState() {
        AllegroRobot2024 robot = (AllegroRobot2024)getRobot().getRobotSubsystem() ;
        SwerveDriveGamepad gp = (SwerveDriveGamepad)getGamePad() ;

        if (startCollectAction_.isDone()) {
            if (robot.getIntakeShooter().isHoldingNote()) {
                dispositionNoteInIntake();
            }
            else {
                state_ = OIState.Idle ;
            }
        }
        else if (!gp.isRBackButtonPressed() && !startCollectAction_.isCollectingNote()) {
            robot.getSuperStructure().setAction(stopCollectAction_) ;
            state_ = OIState.EndingCollect ;
        }
    }

    private void endingCollectState() {
        AllegroRobot2024 robot = (AllegroRobot2024)getRobot().getRobotSubsystem() ;        
        if (stopCollectAction_.isDone()) {
            if (robot.getIntakeShooter().isHoldingNote()) {
                dispositionNoteInIntake();
            }
            else {
                state_ = OIState.Idle ;
            }
        }
    }

    private void waitingToShootState() {
        AllegroRobot2024 robot = (AllegroRobot2024)getRobot().getRobotSubsystem() ;
        NoteTarget target = oipanel_.getNoteTarget() ;
        if (target != NoteTarget.Speaker) {
            //
            // We have switched to a different target, abort the shoot action
            //
            shoot_action_.cancel() ;
            robot.getSuperStructure().setAction(fwd_transfer_action_) ;
            state_ = OIState.TransferToAmpTrap;
        }
        else {
            //
            // We are waiting to shoot.  The user must press the shoot button
            // to actually complete the shoot operation.
            //
            if (oipanel_.isShootPressed()) {
                //
                // Find the desired absolute angle for the robot to aim at the goal.
                //
                getGamePad().disable();
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
        if (!shoot_action_.isShooting() && oipanel_.isAbortPressed()) {
            //
            // The drive team has pressed the abort button before the shot actually occurred.  Cancel the
            // shot and return to the state where they can press the shoot button again.
            //
            state_ = OIState.WaitingToShoot ;
            getGamePad().enable();
        }
        else if (!shoot_action_.isShooting() && oipanel_.getNoteTarget() != NoteTarget.Speaker) {
            //
            // The drive team has changed the target before the shot actually occurred.  Cancel the shot and
            // return deploy the note to the manipulator on the elevator/arm.
            //
            shoot_action_.cancel() ;
            getGamePad().enable();
            dispositionNoteInIntake() ;            
        }
        else if (shoot_action_.isDone()) {
            AllegroRobot2024 robot = (AllegroRobot2024)getRobot().getRobotSubsystem() ;

            //
            // Stop tracking the target if you were
            //
            targetLockMode(false);
            
            //
            // Enable the game pad
            //
            getGamePad().enable();

            //
            // Stop the rotate tracking action
            //
            rotate_action_.cancel() ;

            //
            // Stow the intake after shooting
            //
            robot.getIntakeShooter().setAction(stow_intake_action_, true) ;

            //
            // And back to user control completely
            //
            state_ = OIState.StowIntake ;
        }
    }

    private void stowIntakeState() {
        if (stow_intake_action_.isDone()) {
            state_ = OIState.Idle ;
        }
    }

    private void transferToAmpTrapState() {
        if (fwd_transfer_action_.isDone()) {
            AllegroRobot2024 robot = (AllegroRobot2024)getRobot().getRobotSubsystem() ;
            NoteTarget target = oipanel_.getNoteTarget() ;

            //
            // We have transferred the note to the manipulator on the elevator/arm.
            // Now, decide how to position the elevator/arm based on the target feature
            // (amp or trap).
            //
            if (target == NoteTarget.Amp) {
                state_ = OIState.NoteGoingToAmpPositon ;
                robot.getAmpTrap().setAction(amp_prep_pos_action_, true) ;
            }
            else {
                robot.getAmpTrap().setAction(move_note_action_, true) ;
                state_ = OIState.MoveNoteToBack ;
            }

            //
            // While we are doing this, also stow the intake shooter
            //
            robot.getIntakeShooter().setAction(stow_intake_action_, true) ;
        }
    }

    private void noteGoingToAmpPositonState() {
        if (amp_prep_pos_action_.isDone()) {
            state_ = OIState.NoteInAmpPosition ;
        }
    }

    private void moveNoteToBackState() {
        if (move_note_action_.isDone()) {
            AllegroRobot2024 robot = (AllegroRobot2024)getRobot().getRobotSubsystem() ;            
            state_ = OIState.NoteGoingToTrapPosition ;
            robot.getAmpTrap().setAction(goto_trap_place_pos_action_, true) ;
        }
    }

    private void noteGoingToTrapPositionState() {
        if (goto_trap_place_pos_action_.isDone() && stow_intake_action_.isDone()) {
            state_ = OIState.NoteInTrapPosition ;
        }
    }   

    private void noteInAmpPositonState() {
        if (oipanel_.isShootPressed()) {
            AllegroRobot2024 robot = (AllegroRobot2024)getRobot().getRobotSubsystem() ;
            robot.getAmpTrap().setAction(amp_shoot_action_, true) ;
            state_ = OIState.ShootingAmp ;
        }
    }

    private void shootingAmpState() {
        if (amp_shoot_action_.isDone()) {
            AllegroRobot2024 robot = (AllegroRobot2024)getRobot().getRobotSubsystem() ;
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
        oipanel_.setClimbUpPrepLED(LEDState.ON);
        if (oipanel_.isClimbUpPrepPressed()) {
            AllegroRobot2024 robot = (AllegroRobot2024)getRobot().getRobotSubsystem() ;            
            oipanel_.setClimbUpPrepLED(LEDState.BLINK_FAST);
            robot.getAmpTrap().setAction(goto_climb_pos_action_, true) ;

            state_ = OIState.GoToClimbPosition ;
        }
    }

    private void gotoClimbPositionState() {
        if (goto_climb_pos_action_.isDone()) {
            AllegroRobot2024 robot = (AllegroRobot2024)getRobot().getRobotSubsystem() ;            
            robot.getSuperStructure().getClimber().setAction(hooks_up_action_, true) ;
            state_ = OIState.WaitForHooksUp ;
        }
    }

    private void waitForHooksUpState() {
        if (hooks_up_action_.isDone()) {
            oipanel_.setClimbUpPrepLED(LEDState.OFF);
            oipanel_.setClimbUpExecLED(LEDState.ON) ;
            state_ = OIState.WaitForClimbButton ;
        }
    }

    private void waitForClimbButtonState() {
        if (oipanel_.isAbortPressed()) {
            oipanel_.setClimbUpExecLED(LEDState.OFF);
            AllegroRobot2024 robot = (AllegroRobot2024)getRobot().getRobotSubsystem() ;
            robot.getSuperStructure().getClimber().setAction(hooks_down_, true) ;
            state_ = OIState.WaitForHooksDown ;
            climb_only_ = false ;
        }
        else if (oipanel_.isClimbUpExecPressed()) {
            oipanel_.setClimbUpExecLED(LEDState.BLINK_FAST);
            AllegroRobot2024 robot = (AllegroRobot2024)getRobot().getRobotSubsystem() ;  
            robot.getSuperStructure().getClimber().setAction(hooks_down_with_robot_action_, true) ;
            state_ = OIState.ClimbingUp ;
        }
    }

    private void waitingForHooksDown() {
        if (hooks_down_.isDone()) {
            AllegroRobot2024 robot = (AllegroRobot2024)getRobot().getRobotSubsystem() ;

            if (climb_only_) {
                oipanel_.setClimbDownLED(LEDState.ON);
                state_ = OIState.Climbed ;
            }
            else {
                state_ = OIState.NoteGoingToTrapPosition ;
                robot.getAmpTrap().setAction(goto_trap_place_pos_action_, true) ;
            }
        }
    }

    private void climbingUpState() {
        if (oipanel_.isAbortPressed()) {
            AllegroRobot2024 robot = (AllegroRobot2024)getRobot().getRobotSubsystem() ;            
            robot.getSuperStructure().getClimber().setAction(hooks_up_action_, true) ;
            state_ = OIState.WaitForHooksReset ;
        }
        else if (hooks_down_with_robot_action_.isDone()) {
            oipanel_.setClimbUpExecLED(LEDState.OFF);

            if (climb_only_) {
                oipanel_.setClimbDownLED(LEDState.ON);
                state_ = OIState.Climbed ;
            }
            else {            
                AllegroRobot2024 robot = (AllegroRobot2024)getRobot().getRobotSubsystem() ;  
                robot.getAmpTrap().setAction(goto_trap_up_action_, true) ;
                state_ = OIState.MovingTrapWhileUp ;
            }
        }
    }

    private void waitForHooksResetState() {
        if (hooks_up_action_.isDone()) {
            state_ = OIState.WaitForClimbButton ;
        }
    }

    private void movingTrapWhileUpState() {
        if (goto_trap_up_action_.isDone()) {
            AllegroRobot2024 robot = (AllegroRobot2024)getRobot().getRobotSubsystem() ;  
            robot.getAmpTrap().setAction(trap_shoot_action_, true) ;
            state_ = OIState.ShootingTrap ;
        }
    }

    private void shootingTrapState() {
        if (trap_shoot_action_.isDone()) {
            state_ = OIState.Climbed ;
            oipanel_.setClimbUpExecLED(LEDState.OFF);
            oipanel_.setClimbDownLED(LEDState.ON);
        }
    }

    private void climbedState() {
        if (oipanel_.isClimbDownPressed()) {
            oipanel_.setClimbDownLED(OILed.LEDState.BLINK_FAST);

            AllegroRobot2024 robot = (AllegroRobot2024)getRobot().getRobotSubsystem() ;  
            robot.getAmpTrap().setAction(goto_climb_down_pos_action_, true) ;            
            state_ = OIState.PrepToClimbDown ;
        }   
    }

    private void prepToClimbDownState() {
        if (goto_climb_down_pos_action_.isDone()) {
            AllegroRobot2024 robot = (AllegroRobot2024)getRobot().getRobotSubsystem() ; 
            robot.getSuperStructure().getClimber().setAction(hooks_up_action_, true) ;            
            state_ = OIState.ClimbingDown ;
        }
    }

    private void climbingDownState() {
        if (hooks_up_action_.isDone()) {
            state_ = OIState.Idle ;
        }
    }

    @Override
    public void run() throws Exception {
        super.run();

        OIState prev = state_ ;        

        if (oipanel_.isEjectPressed() && state_ != OIState.Eject) {
            AllegroRobot2024 robot = (AllegroRobot2024)getRobot().getRobotSubsystem() ;
            robot.getSuperStructure().cancelAction();
            
            robot.getIntakeShooter().setAction(intake_eject_action_);
            robot.getAmpTrap().setAction(trap_eject_action_) ;
            state_ = OIState.Eject ;
        }
        else {
            runOIStateMachine();
        }

        if (prev != state_) {
            MessageLogger logger = getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Debug) ;
            logger.add("OI State changed : " + prev.toString() + " -> " + state_.toString()) ;
            logger.endMessage();
        }        
    }

    private void runOIStateMachine() {
        switch(state_) {
        case Eject:
            if (intake_eject_action_.isDone() && trap_eject_action_.isDone()) {
                state_ = OIState.Idle ;
            }
            break ;

        case Collect:
            collectState() ;
            break ;

        case EndingCollect:
            endingCollectState();
            break ;

        case Idle:
            idleState() ;
            break ;

        case WaitingToShoot:
            waitingToShootState() ;
            break;

        case Shooting:
            shootingState() ;
            break ;

        case StowIntake:
            stowIntakeState() ;
            break ;

        case TransferToAmpTrap:
            transferToAmpTrapState() ;
            break ;

        case NoteGoingToAmpPositon:
            noteGoingToAmpPositonState() ;
            break ;

        case MoveNoteToBack:
            moveNoteToBackState() ;
            break ;

        case NoteGoingToTrapPosition:
            noteGoingToTrapPositionState() ;
            break ;

        case NoteInAmpPosition:
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

        case WaitForHooksUp:
            waitForHooksUpState() ;
            break ;

        case WaitForHooksDown:
            waitingForHooksDown() ;
            break ;            

        case WaitForClimbButton:
            waitForClimbButtonState() ;
            break ;

        case WaitForHooksReset:
            waitForHooksResetState() ;
            break ;

        case ClimbingUp:
            climbingUpState() ;
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

        case PrepToClimbDown:
            prepToClimbDownState() ;
            break ;

        case ClimbingDown:
            climbingDownState() ;
            break ;
        }
    }
    
    @Override
    public void postHWInit() throws Exception {
        super.postHWInit();

        state_ = OIState.Idle ;

        AllegroRobot2024 robot = (AllegroRobot2024)getRobot().getRobotSubsystem() ;
        IntakeShooterSubsystem intake = robot.getIntakeShooter() ;
        TargetTrackerSubsystem tracker = robot.getTargetTracker() ;

        double postol = intake.getSettingsValue("actions:auto-shoot:rotational-position-tolerance").getDouble() ;        
        rotate_action_ = new SwerveTrackAngle(robot.getSwerve(), () -> robot.getTargetTracker().getRotation(), postol) ;
        shoot_action_ = new IntakeAutoShootAction(intake, tracker, false, rotate_action_) ;

        fwd_transfer_action_ = new TransferIntakeToTrampAction(robot.getSuperStructure()) ;

        double v1, v2 ;

        v1 = intake.getUpDown().getSettingsValue("targets:stow").getDouble() ;
        v2 = intake.getTilt().getSettingsValue("targets:stow").getDouble() ; 
        stow_intake_action_ = new IntakeGotoNamedPositionAction(intake, v1, v2) ;
        
        amp_prep_pos_action_ = new AmpTrapPositionAction(robot.getAmpTrap(), "actions:amp:pivot", "actions:amp:elevator") ;
        goto_trap_place_pos_action_ = new AmpTrapPositionAction(robot.getAmpTrap(), "actions:trap:pivot", "actions:trap:elevator") ;
        goto_climb_pos_action_ = new AmpTrapPositionAction(robot.getAmpTrap(), "actions:climb:pivot", "actions:climb:elevator") ;
        goto_climb_down_pos_action_ = new AmpTrapPositionAction(robot.getAmpTrap(), "actions:climb-down:pivot", "actions:climb-down:elevator") ;
        goto_trap_up_action_ = new AmpTrapPositionAction(robot.getAmpTrap(), "actions:trap-up:pivot", "actions:trap-up:elevator") ;

        trap_shoot_action_ = new AmpTrapMoveNote(robot.getAmpTrap(), 10.0) ;
        amp_shoot_action_ = new AmpTrapMoveNote(robot.getAmpTrap(), -10.0) ;        

        stow_amp_trap_action_ = new AmpTrapPositionAction(robot.getAmpTrap(), "actions:stow:pivot", "actions:stow:elevator") ;

        hooks_up_action_ = new ClimbAction(robot.getSuperStructure().getClimber(), ClimbAction.ClimbType.HooksUp) ;
        hooks_down_ = new ClimbAction(robot.getSuperStructure().getClimber(), ClimbAction.ClimbType.HooksDown);
        hooks_down_with_robot_action_ = new ClimbAction(robot.getSuperStructure().getClimber(), ClimbAction.ClimbType.HooksDownWithRobot);

        intake_eject_action_ = new IntakeEjectAction(robot.getIntakeShooter());
        trap_eject_action_ = new AmpTrapEjectAction(robot.getAmpTrap());

        startCollectAction_ = new StartCollectAltAction(intake);
        stopCollectAction_ = new StopCollectAltAction(intake);

        move_note_action_ = new AmpTrapMoveNote(robot.getAmpTrap(), 2.0);
    }
}
