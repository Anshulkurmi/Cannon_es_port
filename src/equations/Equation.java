package equations;

import math.Vec3;
import math.JacobianElement;
import math.Mat3; 
import objects.Body;
import shapes.Shape;

/**
 * Equation base class.
 *
 * `a`, `b` and `eps` are {@link https://www8.cs.umu.se/kurser/5DV058/VT15/lectures/SPOOKlabnotes.pdf SPOOK} parameters that default to `0.0`. See {@link https://github.com/schteppe/cannon.js/issues/238#issuecomment-147172327 this exchange} for more details on Cannon's physics implementation.
 */
public class Equation {
    public int id;
    /**
   * Minimum (read: negative max) force to be applied by the constraint.
   */
    public double minForce;
    /**
   * Maximum (read: positive max) force to be applied by the constraint.
   */
    public double maxForce;
    public Body bi;
    public Body bj;
    public Shape si;
    public Shape sj;
    /**
   * SPOOK parameter
   */
    public double a;
    /**
   * SPOOK parameter
   */
    public double b;
    /**
   * SPOOK parameter
   */
    public double eps;
    protected JacobianElement jacobianElementA;
    protected JacobianElement jacobianElementB;
    public boolean enabled;
    /**
   * A number, proportional to the force added to the bodies.
   */
    public double multiplier;

    public static int idCounter = 0;

    public Equation(Body bi, Body bj, double minForce, double maxForce) {
        this.id = idCounter++;
        this.minForce = minForce;
        this.maxForce = maxForce;
        this.bi = bi;
        this.bj = bj;
        this.a = 0.0; // SPOOK parameter
        this.b = 0.0; // SPOOK parameter
        this.eps = 0.0; // SPOOK parameter
        this.jacobianElementA = new JacobianElement();
        this.jacobianElementB = new JacobianElement();
        this.enabled = true;
        this.multiplier = 0.0;

        setSpookParams(1e7, 4, 1 / 60); // Set typical spook params
    }

    // Set SPOOK parameters
    /**
   * Recalculates a, b, and eps.
   *
   * The Equation constructor sets typical SPOOK parameters as such:
   * * `stiffness` = 1e7
   * * `relaxation` = 4
   * * `timeStep`= 1 / 60, _note the hardcoded refresh rate._
   */
    public void setSpookParams(double stiffness, double relaxation, double timeStep) {
        double d = relaxation;
        double k = stiffness;
        double h = timeStep;
        this.a = 4.0 / (h * (1 + 4 * d));
        this.b = (4.0 * d) / (1 + 4 * d);
        this.eps = 4.0 / (h * h * k * (1 + 4 * d));
    }
    
    //default implementation
    public double computeB(double h) {
    	return computeB(0,0,h);
    }

    // Compute right-hand side of the SPOOK equation
    public double computeB(double a, double b, double h) {
        double GW = computeGW();
        double Gq = computeGq();
        double GiMf = computeGiMf();
        return -Gq * a - GW * b - GiMf * h;
    }

    // Compute G*q, where q are the generalized body coordinates
    public double computeGq() {
        JacobianElement GA = this.jacobianElementA;
        JacobianElement GB = this.jacobianElementB;
        Body bi = this.bi;
        Body bj = this.bj;
        Vec3 xi = bi.position;
        Vec3 xj = bj.position;
        return GA.spatial.dot(xi) + GB.spatial.dot(xj);
    }

    // Compute G*W, where W are the body velocities
    public double computeGW() {
        JacobianElement GA = this.jacobianElementA;
        JacobianElement GB = this.jacobianElementB;
        Body bi = this.bi;
        Body bj = this.bj;
        Vec3 vi = bi.velocity;
        Vec3 vj = bj.velocity;
        Vec3 wi = bi.angularVelocity;
        Vec3 wj = bj.angularVelocity;
        return GA.multiplyVectors(vi, wi) + GB.multiplyVectors(vj, wj);
    }

    // Compute G*Wlambda, where W are the body velocities
    public double computeGWlambda() {
        JacobianElement GA = this.jacobianElementA;
        JacobianElement GB = this.jacobianElementB;
        Body bi = this.bi;
        Body bj = this.bj;
        Vec3 vi = bi.vlambda;
        Vec3 vj = bj.vlambda;
        Vec3 wi = bi.wlambda;
        Vec3 wj = bj.wlambda;
        return GA.multiplyVectors(vi, wi) + GB.multiplyVectors(vj, wj);
    }

    // Compute G*inv(M)*f, where M is the mass matrix with diagonal blocks for each body, and f are the forces on the bodies
    public double computeGiMf() {
        JacobianElement GA = this.jacobianElementA;
        JacobianElement GB = this.jacobianElementB;
        Body bi = this.bi;
        Body bj = this.bj;
        Vec3 fi = bi.force;
        Vec3 ti = bi.torque;
        Vec3 fj = bj.wlambda;
        Vec3 tj = bj.torque;
        double invMassi = bi.invMassSolve;
        double invMassj = bj.invMassSolve;

        //added imfi and imfj
        fi.scale(invMassi,iMfi);
        fj.scale(invMassj,iMfj);
        //added invIi_vmult_taui
        bi.invInertiaWorldSolve.vmult(ti,invIi_vmult_taui);
        bj.invInertiaWorldSolve.vmult(tj,invIj_vmult_tauj);

        return GA.multiplyVectors(iMfi, invIi_vmult_taui) + GB.multiplyVectors(iMfj, invIj_vmult_tauj);
    }

    // Compute G*inv(M)*G'
    public double computeGiMGt() {
        JacobianElement GA = this.jacobianElementA;
        JacobianElement GB = this.jacobianElementB;
        Body bi = this.bi;
        Body bj = this.bj;
        double invMassi = bi.invMassSolve;
        double invMassj = bj.invMassSolve;
        Mat3 invIi = bi.invInertiaWorldSolve;
        Mat3 invIj = bj.invInertiaWorldSolve;
        double result = invMassi + invMassj;
        
        //added tmp
        invIi.vmult(GA.rotational,tmp);
        //result += GA.rotational.dot(GA.rotational);
        result += tmp.dot(GA.rotational);
        //changed value of result 
        invIj.vmult(GB.rotational,tmp);
        result += tmp.dot(GB.rotational);

        return result;
    }

    // Add constraint velocity to the bodies
    public void addToWlambda(double deltalambda) {
        JacobianElement GA = this.jacobianElementA;
        JacobianElement GB = this.jacobianElementB;
        Body bi = this.bi;
        Body bj = this.bj;
        Vec3 temp = addToWlambda_temp ; //new Vec3();

        // Add to linear velocity
        //added bi.vlambda as target 
        bi.vlambda.addScaledVector(bi.invMassSolve * deltalambda, GA.spatial,bi.vlambda);
        bj.vlambda.addScaledVector(bj.invMassSolve * deltalambda, GB.spatial,bj.vlambda);

        // Add to angular velocity
        bi.invInertiaWorldSolve.vmult(GA.rotational, temp);
        bi.wlambda.addScaledVector(deltalambda, temp);

        bj.invInertiaWorldSolve.vmult(GB.rotational, temp);
        bj.wlambda.addScaledVector(deltalambda, temp,bj.wlambda);//added bj.wlambda as target 
    }

    // Compute the denominator part of the SPOOK equation: C = G*inv(M)*G' + eps
    public double computeC() {
        return computeGiMGt() + this.eps;
    }

    static Vec3 iMfi = new Vec3() ;
    static Vec3  iMfj = new Vec3();
    static Vec3  invIi_vmult_taui = new Vec3();
    static Vec3  invIj_vmult_tauj = new Vec3();

    static Vec3  tmp = new Vec3();
    static Vec3  addToWlambda_temp = new Vec3();
}

