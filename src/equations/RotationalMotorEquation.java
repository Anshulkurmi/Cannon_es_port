package equations;


import math.Vec3;
import math.JacobianElement;
import objects.Body;


public class RotationalMotorEquation extends Equation {
    public Vec3 axisA; // World oriented rotational axis for body A
    public Vec3 axisB; // World oriented rotational axis for body B
    public double targetVelocity; // Motor velocity
    
    public RotationalMotorEquation(Body bodyA, Body bodyB) {
    	this(bodyA,bodyB,1e6);
    }

    public RotationalMotorEquation(Body bodyA, Body bodyB, double maxForce) {
        super(bodyA, bodyB, -maxForce, maxForce);

        // Initialize rotational axes and target velocity
        this.axisA = new Vec3();
        this.axisB = new Vec3();
        this.targetVelocity = 0;
    }

    @Override
    public double computeB(double h) {
        double a = this.a; // SPOOK parameter
        double b = this.b; // SPOOK parameter
        Body bi = this.bi; // Body A
        Body bj = this.bj; // Body B
        Vec3 axisA = this.axisA; // World-oriented rotational axis for body A
        Vec3 axisB = this.axisB; // World-oriented rotational axis for body B
        JacobianElement GA = this.jacobianElementA; // Jacobian element for body A
        JacobianElement GB = this.jacobianElementB; // Jacobian element for body B

        // Constraint equation: g = 0
        // Relative angular velocity (gdot) = axisA * wi - axisB * wj
        //gdot = G * W = G * [vi wi vj wj]
        // G = [0 axisA 0 -axisB]

        // Set the rotational components of Jacobian elements
        GA.rotational.copy(axisA); // Set GA's rotational element as axisA
        axisB.negate(GB.rotational); // Set GB's rotational element as -axisB

        // Compute the constraint force (B) based on relative angular velocity and max force
        double GW = this.computeGW() - this.targetVelocity; // Relative angular velocity minus target velocity
        double GiMf = this.computeGiMf(); // Compute G * inv(M) * f

        // Compute the constraint force (B)
        double B = -GW * b - h * GiMf;

        return B;
    }
}
