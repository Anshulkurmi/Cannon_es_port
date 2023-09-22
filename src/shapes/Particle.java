package shapes;

import math.Vec3; // Import the appropriate Vec3 class
import math.Quaternion; // Import the appropriate Quaternion class

/**
 * Particle shape.
 */
public class Particle extends Shape {
	
	
	public Particle() {
		this(null);
	}
    /**
     * Constructor for Particle shape.
     */
    public Particle(ShapeOptions options) {
        super(ShapeTypes.PARTICLE,options); // Initialize with the type ShapeType.PARTICLE
    }

    /**
     * Calculate the local inertia of the particle shape.
     *
     * @param mass   The mass of the particle.
     * @param target The vector to store the local inertia.
     * @return The local inertia vector.
     */
    @Override
    public Vec3 calculateLocalInertia(double mass, Vec3 target) {
        target.set(0, 0, 0); // Particle shape has no local inertia
        return target;
    }

    /**
     * Get the volume of the particle shape.
     *
     * @return The volume of the particle (always 0).
     */
    @Override
    public double volume() {
        return 0; // The particle has no volume
    }

    /**
     * Update the bounding sphere radius of the particle shape.
     */
    @Override
    public void updateBoundingSphereRadius() {
        this.boundingSphereRadius = 0; // Bounding sphere radius is 0 for a point particle
    }

    /**
     * Calculate the world axis-aligned bounding box (AABB) of the particle shape.
     *
     * @param pos  The position of the particle in world coordinates.
     * @param quat The orientation of the particle.
     * @param min  The minimum corner of the AABB.
     * @param max  The maximum corner of the AABB.
     */
    @Override
    public void calculateWorldAABB(Vec3 pos, Quaternion quat, Vec3 min, Vec3 max) {
        // The AABB of a point particle is just a point at its position
        min.copy(pos);
        max.copy(pos);
    }
}
