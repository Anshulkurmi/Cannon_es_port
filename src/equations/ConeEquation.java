package equations;

import math.Vec3;
import objects.Body;
import math.JacobianElement;

public class ConeEquation extends Equation {

    // Local axis in A
    public Vec3 axisA;

    // Local axis in B
    public Vec3 axisB;

    // The "cone angle" to keep
    public double angle;
    
    public ConeEquation(Body bodyA,Body bodyB) {
    	this(bodyA,bodyB,new ConeEquationOptions());
    }

    public ConeEquation(Body bodyA, Body bodyB, ConeEquationOptions options) {
        //double maxForce = (options != null && options.maxForce != null) ? options.maxForce : 1e6;
        super(bodyA, bodyB, -options.maxForce, options.maxForce);

        this.axisA = (options != null && options.axisA != null) ? options.axisA.clone() : new Vec3(1, 0, 0);
        this.axisB = (options != null && options.axisB != null) ? options.axisB.clone() : new Vec3(0, 1, 0);
        this.angle = (options != null && options.angle != null) ? options.angle : 0.0;
    }
    
    @Override
    public double computeB(double h) {
    	double a = this.a ;
    	double b = this.b ;
        Vec3 ni = this.axisA;
        Vec3 nj = this.axisB;
        Vec3 nixnj = new Vec3();
        Vec3 njxni = new Vec3();
        JacobianElement GA = this.jacobianElementA;
        JacobianElement GB = this.jacobianElementB;

        // Calculate cross products
        ni.cross(nj, nixnj);
        nj.cross(ni, njxni);

        // The angle between two vectors is:
        // cos(theta) = a * b / (length(a) * length(b)) = { len(a) = len(b) = 1 } = a * b

        // g = a * b
        // gdot = (b x a) * wi + (a x b) * wj
        // G = [0 bxa 0 axb]
        // W = [vi wi vj wj]
        GA.rotational.copy(njxni);
        GB.rotational.copy(nixnj);

        double g = Math.cos(this.angle) - ni.dot(nj);
        double GW = this.computeGW();
        double GiMf = this.computeGiMf();

        return -g * a - GW * b - h * GiMf;
    }
    
    

}
