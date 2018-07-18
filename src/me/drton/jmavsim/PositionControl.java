package me.drton.jmavsim;

public class PositionControl extends WorldObject {

    protected Controller controller;
    protected long startTime;
    protected boolean tookOff = false;

    public PositionControl(World world, Controller controller) {
        super(world);
        this.controller = controller;
        startTime = System.currentTimeMillis();
    }

    /**
     *
     * @param t current time in millisecs
     */
    public void update(long t) {
        double dt = t - startTime - 2000;
        if (dt < 0) {

            // don't do anything before 2 sec
            return;
        }

        if (!tookOff) {
            System.out.println("Trying to take off");
            tookOff = true;
            controller.setSetpoint(0,0,-2,0);
        }

        // spiral
        double z = -10.0 * dt / 60000;
        double alpha = 2 * Math.PI / 15000;
        double radius = 4;
        double psi = alpha + Math.PI / 2;
        double x = radius * Math.cos(alpha) - radius;
        double y = radius * Math.sin(alpha);
//        controller.setSetpoint(x, y, z, psi);

//        if (controller.getVehicle().position.z > -1.9) {
//            controller.setSetpoint(0,0,z,0);
//        } else {
//            controller.setSetpoint(2, 0, -2, 0);
//        }
    }
}
