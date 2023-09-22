package equations;

import math.Vec3;
import math.JacobianElement;
import objects.Body;

public class RotationalEquation extends Equation {
    public Vec3 axisA; // World oriented rotational axis for body A
    public Vec3 axisB; // World oriented rotational axis for body B
    public double maxAngle; // Maximum angle between axes

    public RotationalEquation(Body bodyA, Body bodyB) {
        this(bodyA, bodyB, new RotationalEquationOptions());
    }

    public RotationalEquation(Body bodyA, Body bodyB, RotationalEquationOptions options) {
        // double maxForce = (options != null && options.maxForce != null) ?
        // options.maxForce : 1e6;
        super(bodyA, bodyB, -options.maxForce, options.maxForce);

        // Initialize axisA with the provided axis or default to (1, 0, 0)
        this.axisA = (options != null && options.axisA != null) ? options.axisA.clone() : new Vec3(1, 0, 0);

        // Initialize axisB with the provided axis or default to (0, 1, 0)
        this.axisB = (options != null && options.axisB != null) ? options.axisB.clone() : new Vec3(0, 1, 0);

        // Set the maximum allowed angle between axes to pi/2 radians (90 degrees)
        this.maxAngle = Math.PI / 2;
    }

    @Override
    public double computeB(double h) {
        double a = this.a;
        double b = this.b;
        Vec3 ni = this.axisA;
        Vec3 nj = this.axisB;
        Vec3 nixnj = tmpVec1;
        Vec3 njxni = tmpVec2;
        JacobianElement GA = this.jacobianElementA;
        JacobianElement GB = this.jacobianElementB;

        // Calculate the cross products ni x nj and nj x ni
        ni.cross(nj, nixnj);
        nj.cross(ni, njxni);

        //added
        // g = ni * nj
        // gdot = (nj x ni) * wi + (ni x nj) * wj
        // G = [0 njxni 0 nixnj]
        // W = [vi wi vj wj]
        GA.rotational.copy(njxni);
        GB.rotational.copy(nixnj);

        // Compute the cosine of the angle between the axes
        double g = Math.cos(this.maxAngle) - ni.dot(nj);
        double GW = this.computeGW();
        double GiMf = this.computeGiMf();

        // Calculate the right-hand side (B) of the equation
        double B = -g * a - GW * b - h * GiMf;

        return B;
    }

    // Temporary vectors for calculations
    static  Vec3 tmpVec1 = new Vec3();
    static  Vec3 tmpVec2 = new Vec3();
}


