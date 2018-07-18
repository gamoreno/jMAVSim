package me.drton.jmavsim;

import me.drton.jmavsim.vehicle.AbstractVehicle;

import javax.vecmath.Vector3d;
import java.util.ArrayList;
import java.util.List;
import java.io.BufferedReader;
import java.io.FileReader;
import javax.vecmath.Matrix4d;


public class Controller extends WorldObject {

    protected AbstractVehicle vehicle;
    protected double[][] minusK = new double[4][16];

    protected long lastUpdate = 0;
    protected double[] setpoint = new double[4]; // x, y, z, psi
    protected Integrator[] errorInteg;
    protected double[][] invNormalization = new double[4][4];

    public Controller(World world, AbstractVehicle vehicle) {
        super(world);
        this.vehicle = vehicle;
        errorInteg = new Integrator[4];
        for (int i = 0; i < 4; i++) {
            errorInteg[i] = new Integrator();
        }

        double L = 0.33/2;

        genNormalizationMatrix(4, 0.05, L);
    }

    public AbstractVehicle getVehicle() {
        return vehicle;
    }

    protected void genNormalizationMatrix(double k1, double k2, double L) {
        Matrix4d m = new Matrix4d(
                k1, k1, k1, k1,
                0, -L * k1, 0, L * k1,
                L * k1, 0, -L * k1, 0,
                -k2, k2, -k2, k2
                );
        m.invert();
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                invNormalization[i][j] = m.getElement(i, j);
            }
        }
    }

    public void loadK(String path) throws java.io.FileNotFoundException, java.io.IOException {
        BufferedReader bufferedReader = new BufferedReader(new FileReader(path));
        StringBuffer stringBuffer = new StringBuffer();
        int row = 0;
        String line;
        while ((line = bufferedReader.readLine()) != null) {
            //System.out.println("line:" + line);
            String[] values = line.trim().split("[ \t,]+");
            for (int c = 0; c < values.length; c++) {
                //System.out.print("[" + values[c] + "]");
                minusK[row][c] = -Double.parseDouble(values[c]); // negate to get -K
            }
            //System.out.println();
            row++;
        }
        bufferedReader.close();
    }

    public void printK() {
        for (int r = 0; r < minusK.length; r++) {
            for (int c = 0; c < minusK[r].length; c++) {
                System.out.print(minusK[r][c]);
                System.out.print("\t");
            }
            System.out.println();
        }
    }

    protected void printVector(double[] v, String label) {
        System.out.print(label);
        for (int i = 0; i < v.length; i++) {
            System.out.print(" ");
            System.out.print(v[i]);
        }
        System.out.println();
    }

    public void setSetpoint(double x, double y, double z, double psi) {
        setpoint[0] = x;
        setpoint[1] = y;
        setpoint[2] = z;
        setpoint[3] = psi;

        printVector(setpoint,"Setpoint:");
    }

    /**
     *
     * @param t current time in millisecs
     */
    public void update(long t) {

        /*
         * To get sensors readings (with noise)
         * Sensors sensors = vehicle.getSensors();
         *
         * To get actual state (without noise) use:
         * vehicle.position
         * vehicle.velocity
         * vehicle.rotation
         * vehicle.rotationRate
         * vehicle.attitude
         */

        Vector3d pos = vehicle.position;
        Vector3d vel = vehicle.velocity;
        Vector3d att = vehicle.attitude;
        Vector3d rotationRate = vehicle.rotationRate;

        double[] state = new double[16];
        state[0] = vel.x;
        state[1] = pos.x;
        state[2] = vel.y;
        state[3] = pos.y;
        state[4] = vel.z;
        state[5] = pos.z;
        state[6] = rotationRate.x;
        state[7] = att.x;
        state[8] = rotationRate.y;
        state[9] = att.y;
        state[10] = rotationRate.z;
        state[11] = att.z;

        if (lastUpdate > 0) {
            double dt = (t - lastUpdate) / 1000.0;
            for (int i = 0; i < 3; i++) {
                state[12 + i] = errorInteg[i].step(state[1 + i * 2] - setpoint[i], dt);
            }
            state[15] = errorInteg[3].step(state[11] - setpoint[3], dt);
        }
        lastUpdate = t;

        printVector(state, "State:");

        double[] f = Matrix.multiply(minusK, state);

        double[] u = Matrix.multiply(invNormalization, f);

        printVector(f, "-Kx:");
        printVector(u, "U:");

        List<Double> control = new ArrayList<Double>();
        for (int i = 0; i < 4; ++i) {
            //Control outputs -1 .. 1. Channel assignment depends on the simulated hardware.
            control.add(Math.min(Math.max(u[i], -1.0), 1.0));
        }
        vehicle.setControl(control);
    }
}
