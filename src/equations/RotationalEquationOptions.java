package equations;

import math.Vec3;


public class RotationalEquationOptions {
    public Vec3 axisA; // World oriented rotational axis A
    public Vec3 axisB; // World oriented rotational axis B
    public Double maxAngle; // Maximum allowed angle between axes
    public Double maxForce; // Maximum force applied

    public RotationalEquationOptions() {
        // Default values
        this.axisA = new Vec3(1, 0, 0);
        this.axisB = new Vec3(0, 1, 0);
        this.maxAngle = Math.PI / 2;
        this.maxForce = 1e6;
    }
    
    public RotationalEquationOptions(Vec3 axisA, Vec3 axisB , Double maxAngle , Double maxForce) {
    	this.axisA = axisA ; 
    	this.axisB = axisB ;
    	this.maxAngle = maxAngle ;
    	this.maxForce = maxForce ;
    }

    public Vec3 getAxisA() {
        return axisA;
    }

    public void setAxisA(Vec3 axisA) {
        this.axisA = axisA;
    }

    public Vec3 getAxisB() {
        return axisB;
    }

    public void setAxisB(Vec3 axisB) {
        this.axisB = axisB;
    }

    public Double getMaxAngle() {
        return maxAngle;
    }

    public void setMaxAngle(Double maxAngle) {
        this.maxAngle = maxAngle;
    }

    public Double getMaxForce() {
        return maxForce;
    }

    public void setMaxForce(Double maxForce) {
        this.maxForce = maxForce;
    }
}
