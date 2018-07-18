package me.drton.jmavsim;

public class Integrator {
    protected double integral = 0.0;
    protected double previous = 0.0;

    public Integrator() {

    }

    public double getIntegral() {
        return integral;
    }

    public double step(double value, double dt) {
        integral = integral + 0.5 * (previous + value) * dt;
        previous = value;
        return integral;
    }

}
