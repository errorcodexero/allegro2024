package frc.robot.automodes;

import org.xero1425.base.controllers.AutoController;
import org.xero1425.base.controllers.SwerveTestAutoMode;
import org.xero1425.base.subsystems.motorsubsystem.MCVelocityAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorPowerSequenceAction;

import frc.robot.subsystems.ampTrap.AmpTrapSubsystem;
//import frc.robot.subsystems.ampTrap.Climb;
import frc.robot.subsystems.ampTrap.ClimbAction;
import frc.robot.subsystems.toplevel.AllegroRobot2024;

public class AllegroTestAutoMode extends SwerveTestAutoMode {

    public AllegroTestAutoMode(AutoController ctrl) throws Exception {
        super(ctrl, "Allegro-Test-Mode");

        AllegroRobot2024 robot = (AllegroRobot2024)ctrl.getRobot().getRobotSubsystem() ;
        AmpTrapSubsystem amptrap = robot.getAmpTrap() ;

        if (createTest()) {
            //
            // If this returns true, the test mode was created for the swerve drive related tests
            //
            return ;
        }

        switch (getTestNumber()) {

            /////////////////////////////////////////////////////////////////////////
            //
            // Feeder tests
            //
            /////////////////////////////////////////////////////////////////////////
            case 10:
                break; 

            /////////////////////////////////////////////////////////////////////////
            //
            // Updown tests
            //
            /////////////////////////////////////////////////////////////////////////
            case 20:
                break;             

            /////////////////////////////////////////////////////////////////////////
            //
            // Shooter1 tests
            //
            /////////////////////////////////////////////////////////////////////////
            case 30:
                break;      
                
            /////////////////////////////////////////////////////////////////////////
            //
            // Shooter2 tests
            //
            /////////////////////////////////////////////////////////////////////////
            case 40:
                break;                     

            /////////////////////////////////////////////////////////////////////////
            //
            // Tilt tests
            //
            /////////////////////////////////////////////////////////////////////////
            case 50:
                break;   
                
            /////////////////////////////////////////////////////////////////////////
            //
            // Elevator tests
            //
            /////////////////////////////////////////////////////////////////////////
            case 60:
                if (amptrap != null && amptrap.getElevator() != null) {
                    addSubActionPair(amptrap.getElevator(), new MotorEncoderPowerAction(amptrap.getElevator(), getDouble("power"), getDouble("duration")), true) ;
                }
                break ;

            case 61:
                if (amptrap != null && amptrap.getElevator() != null) {
                    double duration = getDouble("duration") ;
                    double [] times = new double[] { duration, duration, duration, duration, duration } ;
                    double [] powers = new double[] { 0.1, 0.3, 0.5, 0.7, 0.9} ;
                    addSubActionPair(amptrap.getElevator(), new MotorPowerSequenceAction(amptrap.getElevator(), times, powers), true) ;
                }
                break ;                

            case 62:
                if (amptrap != null && amptrap.getElevator() != null) {
                    addSubActionPair(amptrap.getElevator(), new MCVelocityAction(amptrap.getElevator(), "pids:velocity", getDouble("velocity")), true);
                }
                break ;   
                
            /////////////////////////////////////////////////////////////////////////
            //
            // Arm tests
            //
            /////////////////////////////////////////////////////////////////////////
            case 70:
                break;          
                
            /////////////////////////////////////////////////////////////////////////
            //
            // Wrists tests
            //
            /////////////////////////////////////////////////////////////////////////
            case 80:
                break;                 

            /////////////////////////////////////////////////////////////////////////
            //
            // Manipulator tests
            //
            /////////////////////////////////////////////////////////////////////////
            case 90:
                break;    

            /////////////////////////////////////////////////////////////////////////
            //
            // Climber tests
            //
            /////////////////////////////////////////////////////////////////////////'
            case 100: 
                if(amptrap != null && amptrap.getClimber() != null){
                    addSubActionPair(amptrap.getClimber(), new ClimbAction(amptrap), true);
                }
                break;

                
            /////////////////////////////////////////////////////////////////////////
            //
            // Intake-Shooter tests
            //
            /////////////////////////////////////////////////////////////////////////
            case 110:
                break;                  

            /////////////////////////////////////////////////////////////////////////
            //
            // Amp-Trap tests
            //
            /////////////////////////////////////////////////////////////////////////
            case 120:
                break;                 
        }
    }
}
