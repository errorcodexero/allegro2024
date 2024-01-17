package frc.robot.automodes;

import org.xero1425.base.controllers.AutoController;
import org.xero1425.base.controllers.SwerveTestAutoMode;
import org.xero1425.base.subsystems.vision.LimeLightSubsystem;

import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.toplevel.AllegroRobot2024;

public class AllegroTestAutoMode extends SwerveTestAutoMode {

    public AllegroTestAutoMode(AutoController ctrl) throws Exception {
        super(ctrl, "Allegro-Test-Mode");

        AllegroRobot2024 robotsubsystem = (AllegroRobot2024) ctrl.getRobot().getRobotSubsystem();
        IntakeSubsystem intake = robotsubsystem.getIntakeSubsystem();

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
            case 10:
                
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
