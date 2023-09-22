package world;

public class Profile {
    public double solve;
    public double makeContactConstraints;
    public double broadphase;
    public double integrate;
    public double narrowphase;

    // Constructors, getters, and setters as needed
    
    public Profile() {
    	this(0,0,0,0,0);
    }

    public Profile(double solve, double makeContactConstraints, double broadphase, double integrate, double narrowphase) {
        this.solve = solve;
        this.makeContactConstraints = makeContactConstraints;
        this.broadphase = broadphase;
        this.integrate = integrate;
        this.narrowphase = narrowphase;
    }

    // Getter and Setter methods
    // ...
}
