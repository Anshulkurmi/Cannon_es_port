package equations;

import math.Vec3;

public class ConeEquationOptions{
    public Double maxForce; // Use Double to allow null
    public Vec3 axisA;
    public Vec3 axisB;
    public Double angle; // Use Double to allow null
    
    public ConeEquationOptions() {
    	this( 1e6 , new Vec3(1, 0, 0), new Vec3(0,1,0), 0.0);
    }

    public ConeEquationOptions(Double maxForce , Vec3 axisA , Vec3 axisB , Double angle) {
        // Default constructor
    	this.maxForce = maxForce ; 
    	this.axisA = axisA;
    	this.axisB = axisB;
    	this.angle = angle ;	
    }
}