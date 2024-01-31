package frc.robot.subsystems.amp_trap;

import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderSubsystem;

public class AmpTrapSubsystem extends Subsystem{
    private MotorEncoderSubsystem elevator_;
    private MotorEncoderSubsystem arm_;
    private MotorEncoderSubsystem wrist_;
    private MotorEncoderSubsystem manipulator_;
    private MotorEncoderSubsystem climber_;

    public AmpTrapSubsystem(Subsystem parent) throws Exception{
        super(parent, "amp-trap");

        //The elavator moves the elevator up and down. 0 is fully down, and it is calculated in revolutions. 
        elevator_ = new MotorEncoderSubsystem(this, "elevator", false);
        //elevator_.getMotorController().setCurrentLimit(20.0);
        addChild(elevator_);

        //The arm is attached to the top of the elevator, and moves the manipulator and wrist. 0 is fully pointed down, and it is calculated in degrees.  
        arm_ = new MotorEncoderSubsystem(this, "arm", true);
        //arm_.getMotorController().setCurrentLimit(20.0);
        addChild(arm_);

        //The wrist is attached to the end of the arm. It has the manipulator attached to it. 0 is fully pointed down (the manipulator wheels are paralel with the elevator), and it is calculated in degrees.  
        wrist_ = new MotorEncoderSubsystem(this, "wrist", true);
        //wrist_.getMotorController().setCurrentLimit(20.0);
        addChild(wrist_);

        //The manipulator is the two rolling wheels attached to the wrist. They are used for holding and placing the note. They should use a power action, and it is calculated in revolutions. 
        manipulator_ = new MotorEncoderSubsystem(this, "manipulator", false);
        //manipulator_.getMotorController().setCurrentLimit(20.0);
        addChild(manipulator_);

        //The climber is an elevator (I think) with hooks attached to it to climb. 0 is fully down and it is calculated in revolutions.
        climber_ = new MotorEncoderSubsystem(this, "climber", false);
        //climber_.getMotorController().setCurrentLimit(20.0);
        addChild(climber_);
    }

    public MotorEncoderSubsystem getElevator(){
        return elevator_;
    }

    public MotorEncoderSubsystem getArm(){
        return arm_;
    }

    public MotorEncoderSubsystem getWrist(){
        return wrist_;
    }

    public MotorEncoderSubsystem getManipulator(){
        return manipulator_;
    }

    public MotorEncoderSubsystem getClimber(){
        return climber_;
    }


}
