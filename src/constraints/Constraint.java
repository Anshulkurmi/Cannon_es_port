package constraints;

import java.util.ArrayList;
import java.util.List;

import equations.Equation;
import objects.Body;

/**
 * Constraint base class
 */
public class Constraint {
    // Equations to be solved in this constraint
    public List<Equation> equations;
    
    // Body A
    public Body bodyA;
    
    // Body B
    public Body bodyB;
    
    protected int id;
    
    // Set to false if you don't want the bodies to collide when they are connected
    public boolean collideConnected;

    private static int idCounter = 0;

    public Constraint(Body bodyA, Body bodyB) {
    	 // Set default options if not provided
    	this(bodyA,bodyB,new ConstraintOptions(true,true));
    }
    /*
     * @param collideConnected : Set to false if you don't want the bodies to collide when they are connected
     * @default true 
     * @param wakeUpBodies : Set to false if you don't want the bodies to wake up when they are connected.
     * @default true
     */
    public Constraint(Body bodyA, Body bodyB, ConstraintOptions options) {
        this.equations = new ArrayList<>();
        this.bodyA = bodyA;
        this.bodyB = bodyB;
        this.id = Constraint.idCounter++;
        this.collideConnected = options.collideConnected;

        // Wake up the bodies if wakeUpBodies option is true
        if (options.wakeUpBodies) {
            if (bodyA != null) {
                bodyA.wakeUp();
            }
            if (bodyB != null) {
                bodyB.wakeUp();
            }
        }
    }

    // Update method (to be implemented in subclasses)
    //Update all the equations with data
    public void update() {
        throw new UnsupportedOperationException("Method update() not implemented in this Constraint subclass!");
    }

    // Enable all equations in the constraint
    public void enable() {
        List<Equation> eqs = this.equations;
        for (int i = 0; i < eqs.size(); i++) {
            eqs.get(i).enabled = true;
        }
    }

    // Disable all equations in the constraint
    public void disable() {
        List<Equation> eqs = this.equations;
        for (int i = 0; i < eqs.size(); i++) {
            eqs.get(i).enabled = false;
        }
    }
    
   

    // Other methods and properties can be added here as needed
}


