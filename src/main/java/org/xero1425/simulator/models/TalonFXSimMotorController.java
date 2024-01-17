package org.xero1425.simulator.models;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import org.xero1425.base.motors.IMotorController;
import org.xero1425.simulator.engine.SimulationEngine;


public class TalonFXSimMotorController implements ISimMotorController {
    private final static double kTicksPerRev = 2048 ;

    private TalonFX motor_ ;
    private double revs_ ;
    private double velocity_ ;
    private double acceleration_ ;

    private double last_revs_ ;
    private double last_velocity_ ;
    private double ticks_per_volt_per_second_ ;

    public TalonFXSimMotorController(SimulationEngine engine, String bus, int canid, double ticksPerVoltPerSecond) throws Exception {
        IMotorController ctrl = engine.getRobot().getMotorFactory().getMotorController(bus, canid) ;
        if (!(ctrl.getNativeController() instanceof TalonFX)) {
            throw new Exception("motor on bus '" + bus + "', can id " + canid + " is not a TalonFX V6 motor");
        }

        ticks_per_volt_per_second_ = ticksPerVoltPerSecond ;
        motor_ = (TalonFX)ctrl.getNativeController() ;
    }

    @Override
    public void run(double dt) {
        TalonFXSimState state = motor_.getSimState() ;

        revs_ += (ticks_per_volt_per_second_ * state.getMotorVoltage() * dt) / kTicksPerRev ;
        velocity_ = (revs_ - last_revs_) / dt ;
        acceleration_ = (velocity_ - last_velocity_) / dt ;

        last_revs_ = revs_ ;
        last_velocity_ = velocity_ ;

        state.setRawRotorPosition(revs_) ;
        state.setRotorVelocity(velocity_);
        state.setRotorAcceleration(acceleration_);
    }

    @Override
    public double ticksPerRev() {
        return 2048 ;
    }
}
