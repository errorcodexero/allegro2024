package org.xero1425.base.subsystems.oi ;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController ;

import java.util.ArrayList;
import java.util.List;

import org.xero1425.base.misc.XeroTimer;

import edu.wpi.first.wpilibj.DriverStation ;

/// \file

/// \brief a base class for a gamepad.  This class provides access to the various axis, buttons, and POV
/// switches.  It does not provide any mapping to the drivebase.  This is done by derived classes.
public abstract class Gamepad extends OIDevice
{
    public enum Button {
        A,
        B,
        X,
        Y,
        LJoy,
        RJoy,
        RBack,
        LBack,
        RTrigger,
        LTrigger
    }

    public interface Execute {
        void execute() ;
    } ;

    private class ButtonMapping {
        public Button[] buttons_ ;
        public Execute execute_press_ ;
        public Execute execute_release_ ;
        public boolean pressed_ ;

        public ButtonMapping(Button[] buttons, Execute executePress, Execute executeRelease) {
            buttons_ = buttons ;
            execute_press_ = executePress ;
            execute_release_ = executeRelease ;
            pressed_ = false ;
        }
    }

    // Mapping of specific buttons on the controller to java functions
    private List<ButtonMapping> mappings_ ;

    // The XBOX Controller attached
    private XboxController controller_ ;

    // The timer for rumbling
    private XeroTimer timer_ ;
    
    /// \brief Create the new gamepad
    /// \param oi the OI subsystem that owns this device
    /// \param name the name of this device
    /// \param index the index used to access this device in the WPILib.
    public Gamepad(OISubsystem oi, String name, int index) {
        super(oi, name, index) ;

        controller_ = new XboxController(index) ;
        timer_ = null ;
        mappings_ = new ArrayList<ButtonMapping>() ;        
    }


    public void bindButtons(Button[] buttons, Execute press, Execute release) {
        ButtonMapping bm = new ButtonMapping(buttons, press, release) ;
        mappings_.add(bm) ;
    }

    public void bindButton(Button button, Execute press, Execute release) {
        Button[] buttons = { button } ;
        bindButtons(buttons, press, release) ;
    }      

    /// \brief Rumble the gamepad for a fixed magnitude and duraction
    /// \param amount the magnitude of the rumble
    /// \param duration the duration of the rumble
    public void rumble(double amount, double duration) {
        if (timer_ == null) {
            controller_.setRumble(GenericHID.RumbleType.kRightRumble, amount);
            controller_.setRumble(GenericHID.RumbleType.kLeftRumble, amount) ;
            timer_ = new XeroTimer(getSubsystem().getRobot(), "rumble-timer", amount);
            timer_.start() ;
        }
    }

    @Override
    public void disabledProcessing() {
        processRumble();
    }

    /// \brief Compute the state of this Gamepad device.  For the gamepad the
    /// rumble function is processed.
    @Override
    public void computeState() {

        for(ButtonMapping bm : mappings_) {
            boolean pressed = isButtonSequencePressed(bm.buttons_) ;

            if (pressed && !bm.pressed_) {
                bm.pressed_ = true ;
                if (bm.execute_press_ != null)
                    bm.execute_press_.execute() ;
            }
            else if (!pressed && bm.pressed_) {
                bm.pressed_ = false ;
                if (bm.execute_release_ != null)
                    bm.execute_release_.execute() ;
            }
        }
                
        processRumble();
    }

    private boolean isButtonSequencePressed(Button[] buttons) {
        boolean ret = true ;

        if (buttons == null)
            return false ;

        if (buttons != null && buttons.length > 0) {
            for(Button button : buttons) {
                boolean bstate = false ;

                switch(button) {
                    case A: bstate = isAPressed() ; break ;
                    case B: bstate = isBPressed() ; break ;
                    case X: bstate = isXPressed() ; break ;
                    case Y: bstate = isYPressed() ; break ;
                    case LBack: bstate = isLBackButtonPressed() ; break ;
                    case RBack: bstate = isRBackButtonPressed() ; break ;
                    case LJoy: bstate = isLJoyButtonPressed() ; break ;
                    case RJoy: bstate = isRJoyButtonPressed() ; break ;
                    case LTrigger: bstate = isLTriggerPressed() ; break ;
                    case RTrigger: bstate = isRTriggerPressed() ; break ;
                } ;

                if (!bstate) {
                    ret = false ;
                    break ;
                }
            }
        }

        return ret;
    }      

    private void processRumble() {
        if (timer_ != null) {
            if (timer_.isExpired()) {
                controller_.setRumble(GenericHID.RumbleType.kLeftRumble, 0.0) ;
                controller_.setRumble(GenericHID.RumbleType.kRightRumble, 0.0) ;
                timer_ = null ;
            }
        }
    }

    /// \brief Returns true if the right trigger is pressed
    /// \returns true if the right trigger is pressed
    public boolean isRTriggerPressed() {
        boolean ret ;
        
        try {
            ret = DriverStation.getStickAxis(getIndex(), AxisNumber.RTRIGGER.value) > 0.5 ;
        }
        catch(Exception ex) {
            ret = false ;
        }

        return ret ;
    }

    /// \brief Returns true if the right trigger is pressed
    /// \returns true if the right trigger is pressed    
    public boolean isLTriggerPressed() {
        boolean ret ;
        
        try {
            ret = DriverStation.getStickAxis(getIndex(), AxisNumber.LTRIGGER.value) > 0.5 ;
        }
        catch(Exception ex) {
            ret = false ;
        }

        return ret ;
    }    

    /// \brief Returns true if the right trigger is pressed
    /// \returns true if the right trigger is pressed    
    public boolean isAPressed() {
        boolean ret ;

        try {
            ret = DriverStation.getStickButton(getIndex(), ButtonNumber.A.value) ;
        }
        catch(Exception ex) {
            ret = false ;
        }

        return ret ;
    }

    /// \brief Returns true if the right trigger is pressed
    /// \returns true if the right trigger is pressed    
    public boolean isBPressed() {
        boolean ret ;

        try {
            ret = DriverStation.getStickButton(getIndex(), ButtonNumber.B.value) ;
        }
        catch(Exception ex) {
            ret = false ;
        }

        return ret ;
    }

    /// \brief Returns true if the right trigger is pressed
    /// \returns true if the right trigger is pressed    
    public boolean isXPressed() {
        boolean ret ;

        try {
            ret = DriverStation.getStickButton(getIndex(), ButtonNumber.X.value) ;
        }
        catch(Exception ex) {
            ret = false ;
        }

        return ret ;
    }

    /// \brief Returns true if the right trigger is pressed
    /// \returns true if the right trigger is pressed    
    public boolean isYPressed() {
        boolean ret ;

        try {
            ret = DriverStation.getStickButton(getIndex(), ButtonNumber.Y.value) ;
        }
        catch(Exception ex) {
            ret = false ;
        }

        return ret ;
    }

    /// \brief Returns true if the right trigger is pressed
    /// \returns true if the right trigger is pressed    
    public boolean isLJoyButtonPressed() {
        boolean ret ;

        try {
            ret = DriverStation.getStickButton(getIndex(), ButtonNumber.L_JOY.value) ;
        }
        catch(Exception ex) {
            ret = false ;
        }

        return ret ;
    }

    /// \brief Returns true if the right trigger is pressed
    /// \returns true if the right trigger is pressed    
    public boolean isRJoyButtonPressed() {
        boolean ret ;

        try {
            ret = DriverStation.getStickButton(getIndex(), ButtonNumber.R_JOY.value) ;
        }
        catch(Exception ex) {
            ret = false ;
        }

        return ret ;
    }

    /// \brief Returns true if the right trigger is pressed
    /// \returns true if the right trigger is pressed    
    public boolean isRBackButtonPressed() {
        boolean ret ;

        try {
            ret = DriverStation.getStickButton(getIndex(), ButtonNumber.RB.value) ;
        }
        catch(Exception ex) {
            ret = false ;
        }

        return ret ;    
    }

    /// \brief Returns true if the right trigger is pressed
    /// \returns true if the right trigger is pressed    
    public boolean isLBackButtonPressed() {
        boolean ret ;

        try {
            ret = DriverStation.getStickButton(getIndex(), ButtonNumber.LB.value) ;
        }
        catch(Exception ex) {
            ret = false ;
        }

        return ret ;       
    }

    /// \brief Returns the POV angle for the gamepad
    /// \returns the POV angle for the gamepad
    public POVAngle getPOVAngle() {
        int povval ;
        
        try {
            povval = DriverStation.getStickPOV(getIndex(), 0) ;
        }
        catch(Exception ex) {
            povval = -1 ;
        }
        
        return POVAngle.fromInt(povval) ;
    }

    /// \brief The axis numbers on the joystick
    protected enum AxisNumber {
        LEFTX(0),              ///< Left X axis
        LEFTY(1),              ///< Left Y axis
        LTRIGGER(2),           ///< Left Trigger Axis
        RTRIGGER(3),           ///< Right Trigger Axis
        RIGHTX(4),             ///< Right X axis
        RIGHTY(5) ;            ///< Right Y axis

        /// \brief the value of the axis enum
        public final int value ;
        
        private AxisNumber(int value) {
            this.value = value ;
        }
    } ;

    /// \brief buttons on the gamepad
    protected enum ButtonNumber {
        A(1),                  ///< A button
        B(2),                  ///< B button
        X(3),                  ///< X button
        Y(4),                  ///< Y button
        LB(5),                 ///< Left back button
        RB(6),                 ///< Right back button
        BACK(7),               ///< Back button
        START(8),              ///< Start button
        L_JOY(9),              ///< Left joystick button
        R_JOY(10);             ///< Right joystick button

        /// The value of the enum
        public final int value ;

        private ButtonNumber(int value) {
            this.value = value ;
        }        
    } ;

    /// \brief POV angles
    protected enum POVAngle {
        UP(0),                 ///< Up, 0 degrees
        UPRIGHT(45),           ///< UpRight, 45 degrees
        RIGHT(90),             ///< Right, 90 degrees
        DOWNRIGHT(135),        ///< DownRight, 135 degrees
        DOWN(180),               ///< Down, 180 degrees
        DOWNLEFT(225),         ///< DownLeft, 225 degrees
        LEFT(270),             ///< Left, 270 degrees
        UPLEFT(315),           ///< UpLeft, 315 degrees
        NONE(-1) ;             ///< Not pressed in any direction

        /// \brief the value of the enum
        public final int value ;

        private POVAngle(int value) {
            this.value = value ;
        }

        /// \brief convert to an enum from an integer value
        public static POVAngle fromInt(int id) {
            POVAngle[] As = POVAngle.values() ;
            for(int i = 0 ; i < As.length ; i++) {
                if (As[i].value == id)
                    return As[i] ;
            }

            return NONE ;
        }
    } ;    
}