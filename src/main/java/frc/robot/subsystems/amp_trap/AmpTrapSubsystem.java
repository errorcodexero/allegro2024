package frc.robot.subsystems.amp_trap;

import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderSubsystem;

public class AmpTrapSubsystem extends Subsystem {
    // The elavator moves the elevator up and down. 0 is fully down, and it is calculated in revolutions. 
    private MotorEncoderSubsystem elevator_;

    // The arm is attached to the top of the elevator, and moves the manipulator and wrist. 0 is fully pointed down, and it is calculated in degrees.  
    private MotorEncoderSubsystem arm_;

    // The manipulator is the two rolling wheels attached to the wrist. They are used for holding and placing the note. They should use a power action, and it is calculated in revolutions. 
    private MotorEncoderSubsystem manipulator_;

    // If true, the subsystem is holding a note
    private boolean holding_note_ ;

    public AmpTrapSubsystem(Subsystem parent) throws Exception {
        super(parent, "amp-trap");

        elevator_ = new MotorEncoderSubsystem(this, "elevator", false);
        addChild(elevator_);

        arm_ = new MotorEncoderSubsystem(this, "arm", true);
        addChild(arm_);

        // manipulator_ = new MotorEncoderSubsystem(this, "manipulator", false);
        // addChild(manipulator_);

        holding_note_ = false ;
    }

    public boolean isHoldingNote() {
        return holding_note_ ;
    }

    public void setHoldingNote(boolean value) {
        holding_note_ = value ;
    }

    public MotorEncoderSubsystem getElevator() {
        return elevator_;
    }

    public MotorEncoderSubsystem getArm() {
        return arm_;
    }

    public MotorEncoderSubsystem getManipulator() {
        return manipulator_;
    }
}
