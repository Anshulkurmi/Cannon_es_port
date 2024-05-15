package objects;

import java.util.ArrayList;
import java.util.List;

import math.Vec3;

/**
 * Smoothed-particle hydrodynamics system
 * 
 * @todo Make parameters customizable in the constructor
 */
public class SPHSystem {
    private List<Body> particles; // list of particles
    /**
     * Density of the system (kg/m3).
     * 
     * @default 1
     */
    private double density;
    /**
     * Distance below which two particles are considered to be neighbors.
     * It should be adjusted so there are about 15-20 neighbor particles within this
     * radius.
     * 
     * @default 1
     */
    private double smoothingRadius;
    /**
     * @default 1
     */
    private double speedOfSound;
    /**
     * Viscosity of the system.
     * 
     * @default 0.01
     */
    private double viscosity;
    private double eps; // @default 0.000001
    private List<Double> pressures;
    private List<Double> densities;
    private List<List<Body>> neighbors;

    public SPHSystem() {
        // Initialize particle list and default parameters
        this.particles = new ArrayList<>();
        this.density = 1.0;
        this.smoothingRadius = 1.0;
        this.speedOfSound = 1.0;
        this.viscosity = 0.01;
        this.eps = 0.000001;
        
     // Stuff Computed per particle
        this.pressures = new ArrayList<>();
        this.densities = new ArrayList<>();
        this.neighbors = new ArrayList<>();
    }

    /**
     * Add a particle to the system.
     */
    public void add(Body particle) {
        // Add a particle to the system
        particles.add(particle);
        if (neighbors.size() < particles.size()) {
            neighbors.add(new ArrayList<>());
        }
    }

    public void remove(Body particle) {
        // Remove a particle from the system
        int idx = this.particles.indexOf(particle);
        if (idx != -1) {
            this.particles.remove(idx);
            if (this.neighbors.size() > this.particles.size()) {
                this.neighbors.remove(neighbors.size() - 1);
            }
        }
    }

    
    /**
     * Get neighbors within smoothing volume, save in the array neighbors
     */
    public void getNeighbors(Body particle, List<Body> neighborsList) {
        int N = this.particles.size();
        int id = particle.id;
        double R2 = this.smoothingRadius * this.smoothingRadius;
        Vec3 dist = SPHSystem_getNeighbors_dist;

        for (int i = 0; i < N; i++) {
            Body p = this.particles.get(i);
            p.position.vsub(particle.position, dist);
            if (id != p.id && dist.lengthSquared() < R2) {
                neighborsList.add(p);
            }
        }
    }

    public void update() {
        // Update the SPH system for one time step
        int N = this.particles.size();
        Vec3 dist = SPHSystem_update_dist;
        double cs = speedOfSound;
        double eps = this.eps;

        for (int i = 0; i < N; i++) {
            Body p = this.particles.get(i);//current particle
            List<Body> neighborsList = neighbors.get(i);

            // Clear the list of neighbors
            neighborsList.clear();
            this.getNeighbors(p, neighborsList);
            neighborsList.add(this.particles.get(i)); // Add the current particle to its own neighbors
            int numNeighbors = neighborsList.size();
            double sum = 0.0;

            // Accumulate density for the particle
            for (int j = 0; j < numNeighbors; j++) {
                Body neighbor = neighborsList.get(j);
                //System.out.println("Current particle has position %f %f %f\n",objects[id].pos.x(),objects[id].pos.y(),objects[id].pos.z());
                p.position.vsub(neighbor.position, dist);
                double len = dist.length();

                double weight = this.w(len);
                sum += neighbor.mass * weight;
            }

            // Save density and pressure
            this.densities.add( sum);
            this.pressures.add( cs * cs * (this.densities.get(i) - this.density));
        }

        // Calculate forces and update particle positions
        // ... (The force calculation part is not shown in the comment for brevity)
    }

    //added doubt , in source code some code is there not in any methods 

    private double w(double r) {
        // Calculate the weight using the W(r) weight function
        double h = smoothingRadius;
        return (315.0 / (64.0 * Math.PI * Math.pow(h, 9))) * Math.pow((Math.pow(h, 2) - Math.pow(r, 2)), 3);
    }

    private void gradw(Vec3 rVec, Vec3 resultVec) {
        // Calculate gradient of the weight function
        double r = rVec.length();
        double h = smoothingRadius;
        resultVec.set(
                (945.0 / (32.0 * Math.PI * Math.pow(h, 9))) * Math.pow((Math.pow(h, 2) - Math.pow(r, 2)), 2) * rVec.x,
                (945.0 / (32.0 * Math.PI * Math.pow(h, 9))) * Math.pow((Math.pow(h, 2) - Math.pow(r, 2)), 2) * rVec.y,
                (945.0 / (32.0 * Math.PI * Math.pow(h, 9))) * Math.pow((Math.pow(h, 2) - Math.pow(r, 2)), 2) * rVec.z);
    }

    private double nablaw(double r) {
        // Calculate nabla(W)
        double h = smoothingRadius;
        return (945.0 / (32.0 * Math.PI * Math.pow(h, 9)))
                * (Math.pow((Math.pow(h, 2) - Math.pow(r, 2)), 1) * (7 * Math.pow(r, 2) - 3 * Math.pow(h, 2)));
    }

    static Vec3 SPHSystem_getNeighbors_dist = new Vec3();

    // Temp vectors for calculation
    static Vec3 SPHSystem_update_dist = new Vec3();// Relative velocity

    static Vec3 SPHSystem_update_a_pressure = new Vec3();
    static Vec3 SPHSystem_update_a_visc = new Vec3();
    static Vec3 SPHSystem_update_gradW = new Vec3();
    static Vec3 SPHSystem_update_r_vec = new Vec3();
    static Vec3 SPHSystem_update_u = new Vec3();
}
