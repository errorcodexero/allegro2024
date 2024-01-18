package frc.robot.automodes;

import org.xero1425.base.controllers.AutoController;
import org.xero1425.base.controllers.SwerveTestAutoMode;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;

import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.toplevel.AllegroRobot2024;

public class AllegroTestAutoMode extends SwerveTestAutoMode {

    public AllegroTestAutoMode(AutoController ctrl) throws Exception {
        super(ctrl, "Allegro-Test-Mode");

        AllegroRobot2024 robot = (AllegroRobot2024)ctrl.getRobot().getRobotSubsystem() ;
        IntakeSubsystem intake = robot.getIntakeSubsystem() ;

        if (createTest()) {
            //
            // If this returns true, the test mode was created for the swerve drive related tests
            //
            return ;
        }

        switch (getTestNumber()) {
            //////////////////////////////////////////////////////////////////////////////////////
            //
            // Intake test modes
            //
            //////////////////////////////////////////////////////////////////////////////////////

            // Rotate the up/down motor
            case 10:
                addSubActionPair(intake, new MotorEncoderPowerAction(intake, getDouble("power"), getDouble("duration")), true) ;
                break ;

            //////////////////////////////////////////////////////////////////////////////////////
            //
            // Shooter test modes
            //
            //////////////////////////////////////////////////////////////////////////////////////
            case 20:
                break ;

            //////////////////////////////////////////////////////////////////////////////////////
            //
            // Elevator/Arm test modes
            //
            //////////////////////////////////////////////////////////////////////////////////////
            case 30:
                break ;

            //////////////////////////////////////////////////////////////////////////////////////
            //
            // Dog Head testmodes
            //
            //////////////////////////////////////////////////////////////////////////////////////
            case 40:
                break ; 
        }
    }
}
