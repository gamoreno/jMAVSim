package me.drton.jmavsim;

public class VisionSensors extends SimpleSensors {

    private boolean visionUpdated = false;
    private long visionInterval = 20;  // [ms]
    private long visionNext = 0;
	
    @Override
    public void update(long t) {
    	super.update(t);
    	
    	// control vision update rate
        if (visionNext <= t) {
            visionNext = t + visionInterval;
            visionUpdated = true;
        }
    	
    }
    
    @Override
    public boolean isVisionUpdated() {
        boolean res = visionUpdated;
        visionUpdated = false;
        return res;
    }
}
