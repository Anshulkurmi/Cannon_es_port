package collision;

import java.util.ArrayList;
import java.util.List;
import java.util.Arrays;

import math.Vec3;
import objects.Body;
import world.World;
import shapes.*;

/**
 * Axis aligned uniform grid broadphase.
 * 
 * @todo Needs support for more than just planes and spheres.
 */
public class GridBroadphase extends Broadphase {
    // Number of boxes along x, y, and z axes
    private int nx;
    private int ny;
    private int nz;

    // Axis-aligned bounding box minimum and maximum coordinates
    private Vec3 aabbMin; // aabbMin
    private Vec3 aabbMax; // aabbMax

    // Arrays to store bins and their lengths
    private List<Body>[] bins; // bins
    private int[] binLengths; // binLengths

    // Constructor for the GridBroadphase
    /**
     * @param nx Number of boxes along x.
     * @param ny Number of boxes along y.
     * @param nz Number of boxes along z.
     */
    public GridBroadphase(Vec3 aabbMin, Vec3 aabbMax, int nx, int ny, int nz) {
        super();

        this.nx = nx;
        this.ny = ny;
        this.nz = nz;
        this.aabbMin = aabbMin;
        this.aabbMax = aabbMax;

        // Calculate the total number of bins
        int nbins = this.nx * this.ny * this.nz;

        // Check if the number of bins is valid
        if (nbins <= 0) {
            throw new IllegalArgumentException("GridBroadphase: Each dimension's n must be > 0");
        }

        // Initialize arrays for bins and bin lengths
        this.bins = new ArrayList[nbins];
        // Rather than continually resizing arrays (thrashing the memory), just record
        // length and allow them to grow
        this.binLengths = new int[nbins];

        // Initialize each bin and set its length to 0
        for (int i = 0; i < nbins; i++) {
            this.bins[i] = new ArrayList<>();
            this.binLengths[i] = 0;
        }
    }

    // Method to calculate collision pairs in the physics world
    @Override
    public void collisionPairs(World world, List<Body> pairs1, List<Body> pairs2) {
        int N = world.bodies.size();
        List<Body> bodies = world.bodies;
        Vec3 max = this.aabbMax;
        Vec3 min = this.aabbMin;
        int nx = this.nx;
        int ny = this.ny;
        int nz = this.nz;
        int xstep = ny * nz;
        int ystep = nz;
        int zstep = 1;
        double xmax = max.x;
        double ymax = max.y;
        double zmax = max.z;
        double xmin = min.x;
        double ymin = min.y;
        double zmin = min.z;
        double xmult = nx / (xmax - xmin);
        double ymult = ny / (ymax - ymin);
        double zmult = nz / (zmax - zmin);
        double binsizeX = (xmax - xmin) / nx;
        double binsizeY = (ymax - ymin) / ny;
        double binsizeZ = (zmax - zmin) / nz;
        double binRadius = Math.sqrt(binsizeX * binsizeX + binsizeY * binsizeY + binsizeZ * binsizeZ) * 0.5;
        // int[] types = Shape.types;
        int SPHERE = ShapeTypes.SPHERE.getValue();
        int PLANE = ShapeTypes.PLANE.getValue(); // [Shape.PLANE];
        int BOX = ShapeTypes.BOX.getValue(); // types[Shape.BOX];
        int COMPOUND = ShapeTypes.COMPOUND.getValue(); // types[Shape.COMPOUND];
        int CONVEXPOLYHEDRON = ShapeTypes.CONVEXPOLYHEDRON.getValue(); // types[Shape.CONVEXPOLYHEDRON];

        // Reset binLengths array to 0
        Arrays.fill(this.binLengths, 0);

        // Put all bodies into the bins
        for (int i = 0; i < N; i++) {
            Body bi = bodies.get(i);
            Shape si = bi.shapes.get(0);

            switch (si.type) {
                case SPHERE: {
                    Sphere shape = (Sphere) si;
                    // Put in bin
                    // check if overlap with other bins
                    double x = bi.position.x;
                    double y = bi.position.y;
                    double z = bi.position.z;
                    double r = shape.radius;

                    // Add the sphere to bins based on its position and radius
                    addBoxToBins(x - r, y - r, z - r, x + r, y + r, z + r, bi, xmult, ymult, zmult, xstep, ystep,
                            zstep);
                    break;
                }
                case PLANE: {
                    Plane shape = (Plane) si;

                    if (shape.worldNormalNeedsUpdate) {
                        shape.computeWorldNormal(bi.quaternion);
                    }
                    Vec3 planeNormal = shape.worldNormal;

                    // Relative position from origin of plane object to the first bin
                    // Incremented as we iterate through the bins

                    double xreset = xmin + binsizeX * 0.5 - bi.position.x;
                    double yreset = ymin + binsizeY * 0.5 - bi.position.y;
                    double zreset = zmin + binsizeZ * 0.5 - bi.position.z;
                    Vec3 d = new Vec3(xreset, yreset, zreset);

                    // Iterate through bins and add the plane if it intersects with the bin
                    for (int xi = 0, xoff = 0; xi < nx; xi++, xoff += xstep, d.y = yreset, d.x += binsizeX) {
                        for (int yi = 0, yoff = 0; yi < ny; yi++, yoff += ystep, d.z = zreset, d.y += binsizeY) {
                            for (int zi = 0, zoff = 0; zi < nz; zi++, zoff += zstep, d.z += binsizeZ) {
                                if (d.dot(planeNormal) < binRadius) {
                                    int idx = xoff + yoff + zoff;
                                    this.bins[idx].add(bi);
                                    this.binLengths[idx]++;
                                }
                            }
                        }
                    }
                    break;
                }
                default: {
                    if (bi.aabbNeedsUpdate) {
                        bi.updateAABB();
                    }

                    // Add other shapes to bins based on their AABB
                    addBoxToBins(bi.aabb.lowerBound.x, bi.aabb.lowerBound.y, bi.aabb.lowerBound.z,
                            bi.aabb.upperBound.x, bi.aabb.upperBound.y, bi.aabb.upperBound.z, bi, xmult, ymult, zmult,
                            xstep, ystep, zstep);
                    break;
                }
            }
        }

        // Check for collision pairs within each bin
        for (int i = 0; i < this.bins.length; i++) {
            int binLength = this.binLengths[i];
            if (binLength > 1) {
                List<Body> bin = this.bins[i];

                for (int xi = 0; xi < binLength; xi++) {
                    Body bi = bin.get(xi);
                    for (int yi = 0; yi < xi; yi++) {
                        Body bj = bin.get(yi);

                        // Check if broadphase collision is needed and perform intersection test
                        if (this.needBroadphaseCollision(bi, bj)) {
                            this.intersectionTest(bi, bj, pairs1, pairs2);
                        }
                    }
                }
            }
        }
        // for (let zi = 0, zoff=0; zi < nz; zi++, zoff+= zstep) {
        // console.log("layer "+zi);
        // for (let yi = 0, yoff=0; yi < ny; yi++, yoff += ystep) {
        // const row = '';
        // for (let xi = 0, xoff=0; xi < nx; xi++, xoff += xstep) {
        // const idx = xoff + yoff + zoff;
        // row += ' ' + binLengths[idx];
        // }
        // console.log(row);
        // }
        // }
        // Make collision pairs unique
        this.makePairsUnique(pairs1, pairs2);
    }

    // Helper method to add a box to bins based on its coordinates
    private void addBoxToBins(double x0, double y0, double z0, double x1, double y1, double z1, Body bi, double xmult,
            double ymult, double zmult, int xstep, int ystep, int zstep) {

        // Vec3 max = this.aabbMax;
        // Vec3 min = this.aabbMin;
        // int nx = this.nx;
        // int ny = this.ny;
        // int nz = this.nz;
        // int xstep = ny * nz;
        // int ystep = nz;
        // int zstep = 1;
        // double xmax = max.x;
        // double ymax = max.y;
        // double zmax = max.z;
        // double xmin = min.x;
        // double ymin = min.y;
        // double zmin = min.z;
        // double xmult = nx / (xmax - xmin);
        // double ymult = ny / (ymax - ymin);
        // double zmult = nz / (zmax - zmin);

        int xoff0 = (int) ((x0 - this.aabbMin.x) * xmult);
        int yoff0 = (int) ((y0 - this.aabbMin.y) * ymult);
        int zoff0 = (int) ((z0 - this.aabbMin.z) * zmult);
        int xoff1 = (int) Math.ceil((x1 - this.aabbMin.x) * xmult);
        int yoff1 = (int) Math.ceil((y1 - this.aabbMin.y) * ymult);
        int zoff1 = (int) Math.ceil((z1 - this.aabbMin.z) * zmult);

        // Ensure that bin indices are within valid bounds
        xoff0 = Math.max(0, Math.min(nx - 1, xoff0));
        yoff0 = Math.max(0, Math.min(ny - 1, yoff0));
        zoff0 = Math.max(0, Math.min(nz - 1, zoff0));
        xoff1 = Math.max(0, Math.min(nx - 1, xoff1));
        yoff1 = Math.max(0, Math.min(ny - 1, yoff1));
        zoff1 = Math.max(0, Math.min(nz - 1, zoff1));

        xoff0 *= xstep;
        yoff0 *= ystep;
        zoff0 *= zstep;
        xoff1 *= xstep;
        yoff1 *= ystep;
        zoff1 *= zstep;

        // Add the body to the corresponding bins
        for (int xoff = xoff0; xoff <= xoff1; xoff += xstep) {
            for (int yoff = yoff0; yoff <= yoff1; yoff += ystep) {
                for (int zoff = zoff0; zoff <= zoff1; zoff += zstep) {
                    int idx = xoff + yoff + zoff;
                    this.bins[idx].add(bi);
                }
            }
        }
    }

    static Vec3 GridBroadphase_collisionPairs_d = new Vec3();
    static Vec3 GridBroadphase_collisionPairs_binPos = new Vec3();
}
