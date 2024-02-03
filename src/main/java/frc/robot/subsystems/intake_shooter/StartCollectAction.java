package frc.robot.subsystems.intake_shooter;

import org.xero1425.base.subsystems.motorsubsystem.MCMotionMagicAction;
import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.motorsubsystem.MCVelocityAction;

//This begins the collecting process of moving the intake down to the right position and collects the game piece. This is not the only
//code and has a second part, FinishCollectAction. This uses the updown, tilt and feeder motors. This has a state machine which makes 
//tilt motor move first before the updown brings the intake down.

public class StartCollectAction extends Action{
 
    private enum State {
        Idle,
        TiltUp,
        WaitForNote
    }

    private IntakeShooterSubsystem sub_;

    //Defining the collect actions for the different motors
    private MCMotionMagicAction collect_updown_action;

    private MCMotionMagicAction collect_tilt_action;

    private MCVelocityAction collect_feeder_action;

    private State state_;

    public StartCollectAction(IntakeShooterSubsystem sub)throws Exception {
        super(sub.getRobot().getMessageLogger());
        //switches the state to Idle at first where it doewsn't do anything
        state_ = State.Idle;

        sub_ = sub;
        //Initializing the upDown and tilt collect actions with the Motion Magic constructor
        collect_updown_action = new MCMotionMagicAction(sub.getUpDown(), "updown_position", "targets:collect",0.5, 0.5);

        collect_tilt_action = new MCMotionMagicAction(sub.getTilt(), "tilt_position", "targets:collect", 0,0 );
        //Initializing the feeder collect action with the velocity action
        collect_feeder_action = new MCVelocityAction(sub.getFeeder(), "feeder_velocity", "targets:collect");

    }
    @Override
    public void start() throws Exception {
        super.start();
        //Begins to move the Tilt upwards to a safe position. Also begins spinning the feeder motor
        state_ = State.TiltUp;
        sub_.getFeeder().setAction(collect_feeder_action, true);
        sub_.getTilt().setAction(collect_tilt_action, true);
   }
    @Override
    public void run() throws Exception{
        super.run();
       //Defines the sequence of each state and what to do in these states
        switch(state_) {
            case Idle:
                break;
            case TiltUp:
                stateTiltUp();
                break;
            case WaitForNote:
                stateWaitForNote();
                break;
        }
   }
   @Override
    public String toString(int indent) {
        return prefix(indent) + "StartCollectAction";
    }
    private void stateTiltUp() {
       //Checks if the Tilt motor has reached the safe position. If it has, switch to next state and begin moving the intake down
        if (!sub_.getTilt().isBusy()) {
            state_ = State.WaitForNote;
             sub_.getUpDown().setAction(collect_updown_action, true);
             sub_.getTilt().setAction(collect_tilt_action, true);

        }
    }
    private void stateWaitForNote() {
        //This uses the sensor and keeps checking until the sensor detects a note is in place
        if (sub_.isNotePresent()) {
            sub_.getFeeder().cancelAction();
            state_ = State.Idle;
            setDone();
        }
    }

}
