package org.xero1425.simulator.models;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import org.xero1425.base.motors.IMotorController;
import org.xero1425.base.motors.TalonFXMotorController;
import org.xero1425.simulator.engine.SimulationEngine;

public class TalonFXSimMotorController implements ISimMotorController {
    private final static double kTicksPerRev = 2048 ;

    private TalonFXMotorController motor_ ;
    private DCMotor dcmotor_ ;
    private DCMotorSim sim_ ;

    public TalonFXSimMotorController(SimulationEngine engine, String bus, int canid, int count, double gearing, double moment) throws Exception {
        IMotorController ctrl = engine.getRobot().getMotorFactory().getMotorController(bus, canid) ;
        if (!(ctrl.getNativeController() instanceof TalonFX)) {
            throw new Exception("motor on bus '" + bus + "', can id " + canid + " is not a TalonFX V6 motor");
        }

        motor_ = (TalonFXMotorController)ctrl ;
        getState().Orientation = ChassisReference.Clockwise_Positive ;

        dcmotor_ = DCMotor.getFalcon500(count) ;
        sim_ = new DCMotorSim(dcmotor_, gearing, moment) ;
    }

    @Override
    public void run(double dt) {
        TalonFXSimState state = getState() ;
        state.setSupplyVoltage(RobotController.getBatteryVoltage());
        
        sim_.setInputVoltage(state.getMotorVoltage());
        sim_.update(dt) ;

        state.setRawRotorPosition(sim_.getAngularPositionRotations()) ;
        state.setRotorVelocity(sim_.getAngularVelocityRPM() * 60.0) ;
    }

    @Override
    public double ticksPerRev() {
        return kTicksPerRev ;
    }

    private TalonFXSimState getState() {
        return ((TalonFX)motor_.getNativeController()).getSimState() ;
    }
}
