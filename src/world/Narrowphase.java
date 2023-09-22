package world;

import java.util.ArrayList;
import java.util.List;

import collision.AABB;
import collision.Ray;
import equations.ContactEquation;
import equations.FrictionEquation;
import material.ContactMaterial;
import material.Material;
import math.Quaternion;
import math.Transform;
import math.Vec3;
import objects.Body;
import shapes.Box;
import shapes.ConvexPolyhedron;
import shapes.ConvexPolyhedronContactPoint;
import shapes.Cylinder;
import shapes.Heightfield;
import shapes.Particle;
import shapes.Plane;
import shapes.Shape;
import shapes.Sphere;
import shapes.Trimesh;
import utils.Vec3Pool;

/**
 * Helper class for the World. Generates ContactEquations.
 * 
 * @todo Sphere-ConvexPolyhedron contacts
 * @todo Contact reduction
 * @todo should move methods to prototype
 */
public class Narrowphase {
    /**
     * Internal storage of pooled contact points.
     */
    private List<ContactEquation> contactPointPool;
    private List<FrictionEquation> frictionEquationPool;
    private List<ContactEquation> result;
    private List<FrictionEquation> frictionResult;
    /**
     * Pooled vectors.
     */
    private Vec3Pool v3pool;
    private World world;
    private ContactMaterial currentContactMaterial;
    private boolean enableFrictionReduction;

    public Narrowphase(World world) {
        // Initialize contact point and friction equation pools
        this.contactPointPool = new ArrayList<>();
        this.frictionEquationPool = new ArrayList<>();
        this.result = new ArrayList<>();
        this.frictionResult = new ArrayList<>();
        this.v3pool = new Vec3Pool();
        this.world = world;
        this.currentContactMaterial = world.defaultContactMaterial;
        this.enableFrictionReduction = false;
    }

    /**
     * Make a contact object, by using the internal pool or creating a new one.
     */
    public ContactEquation createContactEquation(Body bi, Body bj, Shape si, Shape sj, Shape overrideShapeA,
            Shape overrideShapeB) {
        ContactEquation c;
        if (!this.contactPointPool.isEmpty()) {
            // Reuse a contact equation from the pool
            c = this.contactPointPool.remove(this.contactPointPool.size() - 1);
            c.bi = bi;
            c.bj = bj;
        } else {
            // Create a new contact equation if the pool is empty
            c = new ContactEquation(bi, bj);
        }

        // Check if collision response is enabled for both bodies and shapes
        c.enabled = (bi.collisionResponse && bj.collisionResponse && si.collisionResponse && sj.collisionResponse);

        // Get the current contact material and set restitution
        ContactMaterial cm = this.currentContactMaterial;
        c.restitution = cm.restitution;

        // Set spook parameters for the contact equation
        c.setSpookParams(cm.contactEquationStiffness, cm.contactEquationRelaxation, this.world.dt);

        // Get materials for si and sj, and calculate restitution
        Material matA = si.material != null ? si.material : bi.material;
        Material matB = sj.material != null ? sj.material : bj.material;
        if (matA != null && matB != null && matA.restitution >= 0 && matB.restitution >= 0) {
            c.restitution = (matA.restitution * matB.restitution);
        }

        // Set shapes for the contact equation, with optional overrides
        c.si = (overrideShapeA != null ? overrideShapeA : si);
        c.sj = (overrideShapeB != null ? overrideShapeB : sj);

        return c;
    }

    public boolean createFrictionEquationsFromContact(ContactEquation contactEquation,
            List<FrictionEquation> outArray) {
        Body bodyA = contactEquation.bi;
        Body bodyB = contactEquation.bj;
        Shape shapeA = contactEquation.si;
        Shape shapeB = contactEquation.sj;

        World world = this.world;
        ContactMaterial cm = this.currentContactMaterial;

        // If friction or restitution were specified in the material, use them
        double friction = cm.friction;
        Material matA = shapeA.material != null ? shapeA.material : bodyA.material;
        Material matB = shapeB.material != null ? shapeB.material : bodyB.material;
        if (matA != null && matB != null && matA.friction >= 0 && matB.friction >= 0) {
            friction = matA.friction * matB.friction;
        }

        if (friction > 0) {
            // Create 2 tangent equations
            // Users may provide a force different from global gravity to use when computing
            // contact friction.
            double mug = friction
                    * (world.frictionGravity != null ? world.frictionGravity.length() : world.gravity.length());
            double reducedMass = bodyA.invMass + bodyB.invMass;
            if (reducedMass > 0) {
                reducedMass = 1.0 / reducedMass;
            }
            List<FrictionEquation> pool = this.frictionEquationPool;
            FrictionEquation c1 = pool.size() > 0 ? pool.remove(pool.size() - 1)
                    : new FrictionEquation(bodyA, bodyB, mug * reducedMass);
            FrictionEquation c2 = pool.size() > 0 ? pool.remove(pool.size() - 1)
                    : new FrictionEquation(bodyA, bodyB, mug * reducedMass);

            c1.bi = c2.bi = bodyA;
            c1.bj = c2.bj = bodyB;
            c1.minForce = c2.minForce = -mug * reducedMass;
            c1.maxForce = c2.maxForce = mug * reducedMass;

            // Copy over the relative vectors
            c1.ri.copy(contactEquation.ri);
            c1.rj.copy(contactEquation.rj);
            c2.ri.copy(contactEquation.ri);
            c2.rj.copy(contactEquation.rj);

            // Construct tangents
            contactEquation.ni.tangents(c1.t, c2.t);

            // Set spook params
            c1.setSpookParams(cm.frictionEquationStiffness, cm.frictionEquationRelaxation, world.dt);
            c2.setSpookParams(cm.frictionEquationStiffness, cm.frictionEquationRelaxation, world.dt);

            c1.enabled = c2.enabled = contactEquation.enabled;

            outArray.add(c1);
            outArray.add(c2);

            return true;
        }

        return false;
    }

    /**
     * Take the average N latest contact point on the plane.
     */
    public void createFrictionFromAverage(int numContacts) {
        // The last contactEquation
        ContactEquation c = this.result.get(this.result.size() - 1);

        // Create the result: two "average" friction equations
        if (!this.createFrictionEquationsFromContact(c, this.frictionResult) || numContacts == 1) {
            return;
        }

        FrictionEquation f1 = this.frictionResult.get(this.frictionResult.size() - 2);
        FrictionEquation f2 = this.frictionResult.get(this.frictionResult.size() - 1);

        averageNormal.setZero();
    averageContactPointA.setZero();
    averageContactPointB.setZero();

        Body bodyA = c.bi;
        Body bodyB = c.bj;
        for (int i = 0; i < numContacts; i++) {
            c = this.result.get(this.result.size() - 1 - i);
            if (c.bi != bodyA) {
                c.ni.vadd(averageNormal, averageNormal);
                c.ri.vadd(averageContactPointA, averageContactPointA);
                c.rj.vadd(averageContactPointB, averageContactPointB);
            } else {
                c.ni.vsub(averageNormal, averageNormal);
                c.rj.vadd(averageContactPointA, averageContactPointA);
                c.ri.vadd(averageContactPointB, averageContactPointB);
            }
        }

        double invNumContacts = 1.0 / numContacts;
        averageContactPointA.scale(invNumContacts, f1.ri);
        averageContactPointB.scale(invNumContacts, f1.rj);
        f2.ri.copy(f1.ri); // Should be the same
        f2.rj.copy(f1.rj);
        averageNormal.normalize();
        averageNormal.tangents(f1.t, f2.t);
    }

    /**
     * Generate all contacts between a list of body pairs
     * 
     * @param p1          Array of body indices
     * @param p2          Array of body indices
     * @param result      Array to store generated contacts
     * @param oldcontacts Optional. Array of reusable contact objects
     */
    public void getContacts(List<Body> p1, List<Body> p2, World world, List<ContactEquation> result,
            List<ContactEquation> oldcontacts, List<FrictionEquation> frictionResult,
            List<FrictionEquation> frictionPool) {
        // Save old contact objects
        this.contactPointPool = oldcontacts;
        this.frictionEquationPool = frictionPool;
        this.result = result;
        this.frictionResult = frictionResult;

        Quaternion qi = tmpQuat1;
        Quaternion qj = tmpQuat2;
        Vec3 xi = tmpVec1;
        Vec3 xj = tmpVec2;

        for (int k = 0, N = p1.size(); k != N; k++) {
            // Get current collision bodies
            Body bi = p1.get(k);
            Body bj = p2.get(k);

            // Get contact material
            ContactMaterial bodyContactMaterial = null;
            if (bi.material != null && bj.material != null) {
                bodyContactMaterial = world.getContactMaterial(bi.material, bj.material);
            }

            boolean justTest = (bi.type & Body.KINEMATIC & bj.type & Body.STATIC) != 0 ||
                    (bi.type & Body.STATIC & bj.type & Body.KINEMATIC) != 0 ||
                    (bi.type & Body.KINEMATIC & bj.type & Body.KINEMATIC) != 0;

            for (int i = 0; i < bi.shapes.size(); i++) {
                bi.quaternion.mult(bi.shapeOrientations.get(i), qi);
                bi.quaternion.vmult(bi.shapeOffsets.get(i), xi);
                xi.vadd(bi.position, xi);
                Shape si = bi.shapes.get(i);

                for (int j = 0; j < bj.shapes.size(); j++) {
                    // Compute world transform of shapes
                    bj.quaternion.mult(bj.shapeOrientations.get(j), qj);
                    bj.quaternion.vmult(bj.shapeOffsets.get(j), xj);
                    xj.vadd(bj.position, xj);
                    Shape sj = bj.shapes.get(j);

                    if ((si.collisionFilterMask & sj.collisionFilterGroup & sj.collisionFilterMask
                            & si.collisionFilterGroup) == 0) {
                        continue;
                    }

                    if (xi.distanceTo(xj) > si.boundingSphereRadius + sj.boundingSphereRadius) {
                        continue;
                    }

                    // Get collision material
                    ContactMaterial shapeContactMaterial = null;
                    if (si.material != null && sj.material != null) {
                        shapeContactMaterial = world.getContactMaterial(si.material, sj.material);
                    }

                    this.currentContactMaterial = shapeContactMaterial != null ? shapeContactMaterial
                            : bodyContactMaterial != null ? bodyContactMaterial : world.defaultContactMaterial;

                    // Get contacts
                    // check this ..... added
                    CollisionTypes resolverIndex = si.type.ordinal() < sj.type.ordinal()
                            ? CollisionTypes.values()[si.type.ordinal() | sj.type.ordinal()]
                            : CollisionTypes.values()[sj.type.ordinal() | si.type.ordinal()];
                    CollisionResolver resolver = this.resolvers.get(resolverIndex);
                    if (resolver != null) {
                        boolean retval = false;

                        // TO DO: investigate why sphereParticle and convexParticle
                        // resolvers expect si and sj shapes to be in reverse order
                        // (i.e. larger integer value type first instead of smaller first)
                        if (si.type.ordinal() < sj.type.ordinal()) {
                            retval = resolver.resolve(si, sj, xi, xj, qi, qj, bi, bj, si, sj, justTest, this);
                        } else {
                            retval = resolver.resolve(sj, si, xj, xi, qj, qi, bj, bi, si, sj, justTest, this);
                        }

                        if (retval && justTest) {
                            // Register overlap
                            world.shapeOverlapKeeper.set(si.id, sj.id);
                            world.bodyOverlapKeeper.set(bi.id, bj.id);
                        }
                    }
                }
            }
        }
    }

    public boolean sphereSphere(Sphere si, Sphere sj, Vec3 xi, Vec3 xj, Quaternion qi, Quaternion qj, Body bi, Body bj,
            Shape rsi, Shape rsj, boolean justTest) {
        if (justTest) {
            return xi.distanceSquared(xj) < Math.pow(si.radius + sj.radius, 2);
        }

        // We will have only one contact in this case
        ContactEquation contactEq = createContactEquation(bi, bj, si, sj, rsi, rsj);

        // Contact normal
        xj.vsub(xi, contactEq.ni);
        contactEq.ni.normalize();

        // Contact point locations
        contactEq.ri.copy(contactEq.ni);
        contactEq.rj.copy(contactEq.ni);
        contactEq.ri.scale(si.radius, contactEq.ri);
        contactEq.rj.scale(-sj.radius, contactEq.rj);

        contactEq.ri.vadd(xi, contactEq.ri);
        contactEq.ri.vsub(bi.position, contactEq.ri);

        contactEq.rj.vadd(xj, contactEq.rj);
        contactEq.rj.vsub(bj.position, contactEq.rj);

        result.add(contactEq);

        createFrictionEquationsFromContact(contactEq, frictionResult);
        // check this return type ......changed
        return true;
    }

    public boolean spherePlane(Sphere si, Plane sj, Vec3 xi, Vec3 xj, Quaternion qi, Quaternion qj, Body bi, Body bj,
            Shape rsi, Shape rsj, boolean justTest) {
        // We will have one contact in this case
        ContactEquation r = createContactEquation(bi, bj, si, sj, rsi, rsj);

        // Contact normal
        r.ni.set(0, 0, 1);
        qj.vmult(r.ni, r.ni);
        r.ni.negate(r.ni); // body i is the sphere, flip normal
        r.ni.normalize(); // Needed?

        // Vector from sphere center to contact point
        r.ni.scale(si.radius, r.ri);

        // Project down sphere on plane
        xi.vsub(xj, point_on_plane_to_sphere);
        r.ni.scale(r.ni.dot(point_on_plane_to_sphere), plane_to_sphere_ortho);
        point_on_plane_to_sphere.vsub(plane_to_sphere_ortho, r.rj); // The sphere position projected to plane

        if (-point_on_plane_to_sphere.dot(r.ni) <= si.radius) {
            if (justTest) {
                return true;
            }

            // Make it relative to the body
            Vec3 ri = r.ri;
            Vec3 rj = r.rj;
            ri.vadd(xi, ri);
            ri.vsub(bi.position, ri);
            rj.vadd(xj, rj);
            rj.vsub(bj.position, rj);

            result.add(r);
            createFrictionEquationsFromContact(r, frictionResult);
        }
        return true;
        // check this return type ......changed
    }

    public boolean boxBox(Box si, Box sj, Vec3 xi, Vec3 xj, Quaternion qi, Quaternion qj, Body bi, Body bj, Shape rsi,
            Shape rsj, boolean justTest) {
        si.convexPolyhedronRepresentation.material = si.material;
        sj.convexPolyhedronRepresentation.material = sj.material;
        si.convexPolyhedronRepresentation.collisionResponse = si.collisionResponse;
        sj.convexPolyhedronRepresentation.collisionResponse = sj.collisionResponse;
        return convexConvex(
                si.convexPolyhedronRepresentation,
                sj.convexPolyhedronRepresentation,
                xi,
                xj,
                qi,
                qj, bi, bj, si, sj, justTest,
                /* added new ArrayList for facelistA ,faceListB */ new ArrayList<Integer>(), new ArrayList<Integer>());
    }

    public boolean sphereBox(Sphere si, Box sj, Vec3 xi, Vec3 xj, Quaternion qi, Quaternion qj, Body bi, Body bj,
            Shape rsi, Shape rsj, boolean justTest) {
        Vec3Pool v3pool = this.v3pool;

        // We refer to the box as body j
        // changed
        Vec3[] sides = sphereBox_sides;
        xi.vsub(xj, box_to_sphere);
        sj.getSideNormals(sides, qj);
        double R = si.radius;
        List<Integer> penetrating_sides = new ArrayList<>();

        // Check side (plane) intersections
        boolean found = false;

        // Store the resulting side penetration info
        Vec3 side_ns = sphereBox_side_ns;
        Vec3 side_ns1 = sphereBox_side_ns1;
        Vec3 side_ns2 = sphereBox_side_ns2;
        Double side_h = null;
        int side_penetrations = 0;
        double side_dot1 = 0;
        double side_dot2 = 0;
        Double side_distance = null;
        for (int idx = 0, nsides = sides.length; idx != nsides && !found; idx++) {
            // Get the plane side normal (ns)
            Vec3 ns = sphereBox_ns;
            ns.copy(sides[idx]);

            double h = ns.length();
            ns.normalize();

            // The normal/distance dot product tells which side of the plane we are
            double dot = box_to_sphere.dot(ns);

            if (dot < h + R && dot > 0) {
                // Intersects plane. Now check the other two dimensions
                Vec3 ns1 = sphereBox_ns1;
                Vec3 ns2 = sphereBox_ns2;
                ns1.copy(sides[(idx + 1) % 3]);
                ns2.copy(sides[(idx + 2) % 3]);
                double h1 = ns1.length();
                double h2 = ns2.length();
                ns1.normalize();
                ns2.normalize();
                double dot1 = box_to_sphere.dot(ns1);
                double dot2 = box_to_sphere.dot(ns2);
                if (dot1 < h1 && dot1 > -h1 && dot2 < h2 && dot2 > -h2) {
                    double dist = Math.abs(dot - h - R);
                    if (side_distance == null || dist < side_distance) {
                        side_distance = dist;
                        side_dot1 = dot1;
                        side_dot2 = dot2;
                        side_h = h;
                        side_ns.copy(ns);
                        side_ns1.copy(ns1);
                        side_ns2.copy(ns2);
                        side_penetrations++;

                        if (justTest) {
                            return true;
                        }
                    }
                }
            }
        }
        if (side_penetrations > 0) {
            found = true;
            ContactEquation r = this.createContactEquation(bi, bj, si, sj, rsi, rsj);
            side_ns.scale(-R, r.ri); // Sphere r
            r.ni.copy(side_ns);
            r.ni.negate(r.ni); // Normal should be out of sphere
            side_ns.scale(side_h, side_ns);
            side_ns1.scale(side_dot1, side_ns1);
            side_ns.vadd(side_ns1, side_ns);
            side_ns2.scale(side_dot2, side_ns2);
            side_ns.vadd(side_ns2, r.rj);

            // Make relative to bodies
            r.ri.vadd(xi, r.ri);
            r.ri.vsub(bi.position, r.ri);
            r.rj.vadd(xj, r.rj);
            r.rj.vsub(bj.position, r.rj);

            this.result.add(r);
            this.createFrictionEquationsFromContact(r, this.frictionResult);
        }

        // Check corners
        Vec3 rj = v3pool.get();
        Vec3 sphere_to_corner = sphereBox_sphere_to_corner;
        for (int j = 0; j != 2 && !found; j++) {
            for (int k = 0; k != 2 && !found; k++) {
                for (int l = 0; l != 2 && !found; l++) {
                    rj.set(0, 0, 0);
                    if (j != 0) {
                        rj.vadd(sides[0], rj);
                    } else {
                        rj.vsub(sides[0], rj);
                    }
                    if (k != 0) {
                        rj.vadd(sides[1], rj);
                    } else {
                        rj.vsub(sides[1], rj);
                    }
                    if (l != 0) {
                        rj.vadd(sides[2], rj);
                    } else {
                        rj.vsub(sides[2], rj);
                    }

                    // World position of corner
                    xj.vadd(rj, sphere_to_corner);
                    sphere_to_corner.vsub(xi, sphere_to_corner);

                    if (sphere_to_corner.lengthSquared() < R * R) {
                        if (justTest) {
                            return true;
                        }
                        found = true;
                        ContactEquation res = this.createContactEquation(bi, bj, si, sj, rsi, rsj);
                        res.ri.copy(sphere_to_corner);
                        res.ri.normalize();
                        res.ni.copy(res.ri);
                        res.ri.scale(R, res.ri);
                        res.rj.copy(rj);

                        // Make relative to bodies
                        res.ri.vadd(xi, res.ri);
                        res.ri.vsub(bi.position, res.ri);
                        res.rj.vadd(xj, res.rj);
                        res.rj.vsub(bj.position, res.rj);

                        this.result.add(res);
                        this.createFrictionEquationsFromContact(res, this.frictionResult);
                    }
                }
            }
        }
        v3pool.release(rj);
        rj = null;

        // Check edges
        Vec3 edgeTangent = v3pool.get();
        Vec3 edgeCenter = v3pool.get();
        Vec3 r = v3pool.get(); // r = edge center to sphere center
        Vec3 orthogonal = v3pool.get();
        Vec3 dist = v3pool.get();
        int Nsides = sides.length;
        for (int j = 0; j != Nsides && !found; j++) {
            for (int k = 0; k != Nsides && !found; k++) {
                if (j % 3 != k % 3) {
                    // Get edge tangent
                    sides[k].cross(sides[j], edgeTangent);
                    edgeTangent.normalize();
                    sides[j].vadd(sides[k], edgeCenter);
                    r.copy(xi);
                    r.vsub(edgeCenter, r);
                    r.vsub(xj, r);
                    double orthonorm = r.dot(edgeTangent); // distance from edge center to sphere center in the tangent
                                                           // direction
                    edgeTangent.scale(orthonorm, orthogonal); // Vector from edge center to sphere center in the tangent
                                                              // direction

                    // Find the third side orthogonal to this one
                    int l = 0;
                    while (l == j % 3 || l == k % 3) {
                        l++;
                    }

                    // vec from edge center to sphere projected to the plane orthogonal to the edge
                    // tangent
                    dist.copy(xi);
                    dist.vsub(orthogonal, dist);
                    dist.vsub(edgeCenter, dist);
                    dist.vsub(xj, dist);

                    // Distances in tangent direction and distance in the plane orthogonal to it
                    double tdist = Math.abs(orthonorm);
                    double ndist = dist.length();

                    if (tdist < sides[l].length() && ndist < R) {
                        if (justTest) {
                            return true;
                        }
                        found = true;
                        ContactEquation res = this.createContactEquation(bi, bj, si, sj, rsi, rsj);
                        edgeCenter.vadd(orthogonal, res.rj); // box rj
                        res.rj.copy(res.rj);
                        dist.negate(res.ni);
                        res.ni.normalize();

                        res.ri.copy(res.rj);
                        res.ri.vadd(xj, res.ri);
                        res.ri.vsub(xi, res.ri);
                        res.ri.normalize();
                        res.ri.scale(R, res.ri);

                        // Make relative to bodies
                        res.ri.vadd(xi, res.ri);
                        res.ri.vsub(bi.position, res.ri);
                        res.rj.vadd(xj, res.rj);
                        res.rj.vsub(bj.position, res.rj);

                        this.result.add(res);
                        this.createFrictionEquationsFromContact(res, this.frictionResult);
                    }
                }
            }
        }
        //v3pool.release(edgeTangent, edgeCenter, r, orthogonal, dist);
        v3pool.release(edgeTangent);
        v3pool.release(edgeCenter);
        v3pool.release(r);
        v3pool.release(orthogonal);
        v3pool.release(dist);
        return found;
    }

    public boolean planeBox(Plane si, Box sj, Vec3 xi, Vec3 xj, Quaternion qi, Quaternion qj, Body bi, Body bj,
            Shape rsi, Shape rsj, boolean justTest) {
        sj.convexPolyhedronRepresentation.material = sj.material;
        sj.convexPolyhedronRepresentation.collisionResponse = sj.collisionResponse;
        sj.convexPolyhedronRepresentation.id = sj.id;
        return this.planeConvex(si, sj.convexPolyhedronRepresentation, xi, xj, qi, qj, bi, bj, si, sj, justTest);
    }

    public boolean convexConvex(ConvexPolyhedron si, ConvexPolyhedron sj, Vec3 xi, Vec3 xj, Quaternion qi,
            Quaternion qj, Body bi, Body bj, Shape rsi, Shape rsj, boolean justTest, List<Integer> faceListA,
            List<Integer> faceListB) {
        Vec3 sepAxis = convexConvex_sepAxis;

        if (xi.distanceTo(xj) > si.boundingSphereRadius + sj.boundingSphereRadius) {
            return false;
        }

        if (si.findSeparatingAxis(sj, xi, qi, xj, qj, sepAxis, faceListA, faceListB)) {
            // added class ConvexPolyhedronContactPoint.java
            List<ConvexPolyhedronContactPoint> res = new ArrayList<>();
            Vec3 q = convexConvex_q;
            si.clipAgainstHull(xi, qi, sj, xj, qj, sepAxis, -100, 100, res);
            int numContacts = 0;
            for (int j = 0; j < res.size(); j++) {
                if (justTest) {
                    return true;
                }
                ContactEquation r = this.createContactEquation(bi, bj, si, sj, rsi, rsj);
                Vec3 ri = r.ri;
                Vec3 rj = r.rj;
                sepAxis.negate(r.ni);
                res.get(j).normal.negate(q);
                q.scale(res.get(j).depth, q);
                res.get(j).point.vadd(q, ri);
                rj.copy(res.get(j).point);

                // Contact points are in world coordinates. Transform back to relative
                ri.vsub(xi, ri);
                rj.vsub(xj, rj);

                // Make relative to bodies
                ri.vadd(xi, ri);
                ri.vsub(bi.position, ri);
                rj.vadd(xj, rj);
                rj.vsub(bj.position, rj);

                this.result.add(r);
                numContacts++;
                if (!this.enableFrictionReduction) {
                    this.createFrictionEquationsFromContact(r, this.frictionResult);
                }
            }
            if (this.enableFrictionReduction && numContacts > 0) {
                this.createFrictionFromAverage(numContacts);
            }
        }
        return false;
    }

    public boolean sphereConvex(Sphere si, ConvexPolyhedron sj, Vec3 xi, Vec3 xj, Quaternion qi, Quaternion qj, Body bi,
            Body bj, Shape rsi, Shape rsj, boolean justTest) {
        // changed Vec3 to Vec3Pool
        Vec3Pool v3pool = this.v3pool;
        xi.vsub(xj, convex_to_sphere);
        List<Vec3> normals = sj.faceNormals;
        List<int[]> faces = sj.faces;
        List<Vec3> verts = sj.vertices;
        double R = si.radius;
        List<Integer> penetrating_sides = new ArrayList<>();

        // if(convex_to_sphere.lengthSquared() > si.boundingSphereRadius +
        // sj.boundingSphereRadius){
        // return;
        // }
        boolean found = false;

        // Check corners
        for (int i = 0; i < verts.size(); i++) {
            Vec3 v = verts.get(i);

            // World position of corner
            Vec3 worldCorner = sphereConvex_worldCorner;
            qj.vmult(v, worldCorner);
            worldCorner.vadd(xj, worldCorner);
            Vec3 sphere_to_corner = sphereConvex_sphereToCorner;
            worldCorner.vsub(xi, sphere_to_corner);
            if (sphere_to_corner.lengthSquared() < R * R) {
                if (justTest) {
                    return true;
                }
                found = true;
                ContactEquation r = this.createContactEquation(bi, bj, si, sj, rsi, rsj);
                r.ri.copy(sphere_to_corner);
                r.ri.normalize();
                r.ni.copy(r.ri);
                r.ri.scale(R, r.ri);
                worldCorner.vsub(xj, r.rj);

                // Should be relative to the body.
                r.ri.vadd(xi, r.ri);
                r.ri.vsub(bi.position, r.ri);

                // Should be relative to the body.
                r.rj.vadd(xj, r.rj);
                r.rj.vsub(bj.position, r.rj);

                this.result.add(r);
                this.createFrictionEquationsFromContact(r, this.frictionResult);
                return true;
            }
        }

        // Check side (plane) intersections
        for (int i = 0, nfaces = faces.size(); i < nfaces && !found; i++) {
            Vec3 normal = normals.get(i);
            int[] face = faces.get(i);

            // Get world-transformed normal of the face
            Vec3 worldNormal = sphereConvex_worldNormal;
            qj.vmult(normal, worldNormal);

            // Get a world vertex from the face
            Vec3 worldPoint = sphereConvex_worldPoint;
            qj.vmult(verts.get(face[0]), worldPoint);
            worldPoint.vadd(xj, worldPoint);

            // Get a point on the sphere, closest to the face normal
            Vec3 worldSpherePointClosestToPlane = sphereConvex_worldSpherePointClosestToPlane;
            worldNormal.scale(-R, worldSpherePointClosestToPlane);
            xi.vadd(worldSpherePointClosestToPlane, worldSpherePointClosestToPlane);

            // Vector from a face point to the closest point on the sphere
            Vec3 penetrationVec = sphereConvex_penetrationVec;
            worldSpherePointClosestToPlane.vsub(worldPoint, penetrationVec);

            // The penetration. Negative value means overlap.
            double penetration = penetrationVec.dot(worldNormal);

            Vec3 worldPointToSphere = sphereConvex_sphereToWorldPoint;
            xi.vsub(worldPoint, worldPointToSphere);

            if (penetration < 0 && worldPointToSphere.dot(worldNormal) > 0) {
                // Intersects plane. Now check if the sphere is inside the face polygon
                List<Vec3> faceVerts = new ArrayList<>(); // Face vertices, in world coords
                for (int j = 0, Nverts = face.length; j < Nverts; j++) {
                    Vec3 worldVertex = v3pool.get();
                    qj.vmult(verts.get(face[j]), worldVertex);
                    xj.vadd(worldVertex, worldVertex);
                    faceVerts.add(worldVertex);
                }

                if (pointInPolygon(faceVerts, worldNormal, xi)) {
                    // Is the sphere center in the face polygon?
                    if (justTest) {
                        return true;
                    }
                    found = true;
                    ContactEquation r = this.createContactEquation(bi, bj, si, sj, rsi, rsj);

                    worldNormal.scale(-R, r.ri); // Contact offset, from sphere center to contact
                    worldNormal.negate(r.ni); // Normal pointing out of sphere

                    Vec3 penetrationVec2 = v3pool.get();
                    worldNormal.scale(-penetration, penetrationVec2);
                    Vec3 penetrationSpherePoint = v3pool.get();
                    worldNormal.scale(-R, penetrationSpherePoint);

                    xi.vsub(xj, r.rj);
                    r.rj.vadd(penetrationSpherePoint, r.rj);
                    r.rj.vadd(penetrationVec2, r.rj);

                    // Should be relative to the body.
                    r.rj.vadd(xj, r.rj);
                    r.rj.vsub(bj.position, r.rj);

                    // Should be relative to the body.
                    r.ri.vadd(xi, r.ri);
                    r.ri.vsub(bi.position, r.ri);

                    v3pool.release(/* added */penetrationVec2);
                    v3pool.release(penetrationSpherePoint);

                    this.result.add(r);
                    this.createFrictionEquationsFromContact(r, this.frictionResult);

                    // Release world vertices
                    for (int j = 0, Nfaceverts = faceVerts.size(); j < Nfaceverts; j++) {
                        v3pool.release(faceVerts.get(j));
                    }

                    return true; // We only expect *one* face contact
                } else {
                    // Edge?
                    for (int j = 0; j < face.length; j++) {
                        // Get two world transformed vertices
                        Vec3 v1 = v3pool.get();
                        Vec3 v2 = v3pool.get();
                        qj.vmult(verts.get(face[(j + 1) % face.length]), v1);
                        qj.vmult(verts.get(face[(j + 2) % face.length]), v2);
                        xj.vadd(v1, v1);
                        xj.vadd(v2, v2);

                        // Construct edge vector
                        Vec3 edge = sphereConvex_edge;
                        v2.vsub(v1, edge);

                        // Construct the same vector, but normalized
                        Vec3 edgeUnit = sphereConvex_edgeUnit;
                        edge.unit(edgeUnit);

                        // p is xi projected onto the edge
                        Vec3 p = v3pool.get();
                        Vec3 v1_to_xi = v3pool.get();
                        xi.vsub(v1, v1_to_xi);
                        double dot = v1_to_xi.dot(edgeUnit);
                        edgeUnit.scale(dot, p);
                        p.vadd(v1, p);

                        // Compute a vector from p to the center of the sphere
                        Vec3 xi_to_p = v3pool.get();
                        p.vsub(xi, xi_to_p);

                        // Collision if the edge-sphere distance is less than the radius
                        // AND if p is in between v1 and v2
                        if (dot > 0 && dot * dot < edge.lengthSquared() && xi_to_p.lengthSquared() < R * R) {
                            // Collision if the edge-sphere distance is less than the radius
                            // Edge contact!
                            if (justTest) {
                                return true;
                            }
                            ContactEquation r = this.createContactEquation(bi, bj, si, sj, rsi, rsj);
                            p.vsub(xj, r.rj);

                            p.vsub(xi, r.ni);
                            r.ni.normalize();

                            r.ni.scale(R, r.ri);

                            // Should be relative to the body.
                            r.rj.vadd(xj, r.rj);
                            r.rj.vsub(bj.position, r.rj);

                            // Should be relative to the body.
                            r.ri.vadd(xi, r.ri);
                            r.ri.vsub(bi.position, r.ri);

                            this.result.add(r);
                            this.createFrictionEquationsFromContact(r, this.frictionResult);

                            // Release world vertices
                            for (int k = 0; k < faceVerts.size(); k++) {
                                v3pool.release(faceVerts.get(k));
                            }

                            v3pool.release(v1);
                            v3pool.release(v2);
                            v3pool.release(p);
                            v3pool.release(xi_to_p);
                            v3pool.release(v1_to_xi);

                            return;
                        }

                        v3pool.release(v1);
                        v3pool.release(v2);
                        v3pool.release(p);
                        v3pool.release(xi_to_p);
                        v3pool.release(v1_to_xi);
                    }
                }

                // Release world vertices
                //added why 2 times 
                for (int k = 0; k < faceVerts.size(); k++) {
                    v3pool.release(faceVerts.get(k));
                }
            }
        }
        return false;
    }

    public boolean planeConvex(Plane planeShape, ConvexPolyhedron convexShape, Vec3 planePosition, Vec3 convexPosition,Quaternion planeQuat, Quaternion convexQuat, Body planeBody, Body convexBody, Shape si, Shape sj,boolean justTest) {
        // Simply return the points behind the plane.
        Vec3 worldVertex = planeConvex_v;

        Vec3 worldNormal = planeConvex_normal;
        worldNormal.set(0, 0, 1);
        planeQuat.vmult(worldNormal, worldNormal); // Turn normal according to plane orientation

        int numContacts = 0;
        Vec3 relpos = planeConvex_relpos;
        for (int i = 0; i < convexShape.vertices.size(); i++) {
            // Get world convex vertex
            worldVertex.copy(convexShape.vertices.get(i));
            convexQuat.vmult(worldVertex, worldVertex);
            convexPosition.vadd(worldVertex, worldVertex);
            worldVertex.vsub(planePosition, relpos);

            double dot = worldNormal.dot(relpos);
            if (dot <= 0.0) {
                if (justTest) {
                    return true;
                }

                ContactEquation r = this.createContactEquation(planeBody, convexBody, planeShape, convexShape, si, sj);

                // Get vertex position projected on plane
                Vec3 projected = planeConvex_projected;
                worldNormal.scale(worldNormal.dot(relpos), projected);
                worldVertex.vsub(projected, projected);
                projected.vsub(planePosition, r.ri); // From plane to vertex projected on plane

                r.ni.copy(worldNormal); // Contact normal is the plane normal out from plane

                // rj is now just the vector from the convex center to the vertex
                worldVertex.vsub(convexPosition, r.rj);

                // Make it relative to the body
                r.ri.vadd(planePosition, r.ri);
                r.ri.vsub(planeBody.position, r.ri);
                r.rj.vadd(convexPosition, r.rj);
                r.rj.vsub(convexBody.position, r.rj);

                this.result.add(r);
                numContacts++;
                if (!this.enableFrictionReduction) {
                    this.createFrictionEquationsFromContact(r, this.frictionResult);
                }
            }
        }

        if (this.enableFrictionReduction && numContacts > 0) {
            this.createFrictionFromAverage(numContacts);
        }
        return false;
    }

    public boolean boxConvex(Box si,ConvexPolyhedron sj,Vec3 xi,Vec3 xj,Quaternion qi,Quaternion qj,Body bi,Body bj,Shape rsi,Shape rsj,boolean justTest) {
        si.convexPolyhedronRepresentation.material = si.material;
        si.convexPolyhedronRepresentation.collisionResponse = si.collisionResponse;
        return this.convexConvex(si.convexPolyhedronRepresentation, sj, xi, xj, qi, qj, bi, bj, rsi, rsj, justTest, /*added */new ArrayList<Integer>(), new ArrayList<Integer>());
    }

    public boolean sphereHeightfield(Sphere sphereShape, Heightfield hfShape, Vec3 spherePos, Vec3 hfPos, Quaternion sphereQuat, Quaternion hfQuat, Body sphereBody, Body hfBody, Shape rsi, Shape rsj,boolean justTest) {
        double[][] data = hfShape.data;
        double radius = sphereShape.radius;
        double w = hfShape.elementSize;
        Vec3 worldPillarOffset = sphereHeightfield_tmp2;

        // Get sphere position to heightfield local!
        Vec3 localSpherePos = sphereHeightfield_tmp1;
        Transform.pointToLocalFrame(hfPos, hfQuat, spherePos, localSpherePos);

        // Get the index of the data points to test against
        int iMinX = (int) Math.floor((localSpherePos.x - radius) / w) - 1;
        int iMaxX = (int) Math.ceil((localSpherePos.x + radius) / w) + 1;
        int iMinY = (int) Math.floor((localSpherePos.y - radius) / w) - 1;
        int iMaxY = (int) Math.ceil((localSpherePos.y + radius) / w) + 1;

        // Bail out if we are out of the terrain
        if (iMaxX < 0 || iMaxY < 0 || iMinX >= data.length || iMinY >= data[0].length) {
            return false;
        }

        // Clamp index to edges
        if (iMinX < 0) {
            iMinX = 0;
        }
        if (iMaxX < 0) {
            iMaxX = 0;
        }
        if (iMinY < 0) {
            iMinY = 0;
        }
        if (iMaxY < 0) {
            iMaxY = 0;
        }
        if (iMinX >= data.length) {
            iMinX = data.length - 1;
        }
        if (iMaxX >= data.length) {
            iMaxX = data.length - 1;
        }
        if (iMaxY >= data[0].length) {
            iMaxY = data[0].length - 1;
        }
        if (iMinY >= data[0].length) {
            iMinY = data[0].length - 1;
        }

        double[] minMax = new double[2];
        hfShape.getRectMinMax(iMinX, iMinY, iMaxX, iMaxY, minMax);
        double min = minMax[0];
        double max = minMax[1];

        // Bail out if we can't touch the bounding height box
        if (localSpherePos.z - radius > max || localSpherePos.z + radius < min) {
            return false;
        }

        List<ContactEquation> result = this.result;
        for (int i = iMinX; i < iMaxX; i++) {
            for (int j = iMinY; j < iMaxY; j++) {
                int numContactsBefore = result.size();
                boolean intersecting = false;

                // Lower triangle
                hfShape.getConvexTrianglePillar(i, j, false);
                Transform.pointToWorldFrame(hfPos, hfQuat, hfShape.pillarOffset, worldPillarOffset);
                if (spherePos.distanceTo(worldPillarOffset) < hfShape.pillarConvex.boundingSphereRadius
                        + sphereShape.boundingSphereRadius) {
                    intersecting = this.sphereConvex(
                            sphereShape,
                            hfShape.pillarConvex,
                            spherePos,
                            worldPillarOffset,
                            sphereQuat,
                            hfQuat,
                            sphereBody,
                            hfBody,
                            rsi,
                            rsj,
                            justTest);
                }

                if (justTest && intersecting) {
                    return true;
                }

                // Upper triangle
                hfShape.getConvexTrianglePillar(i, j, true);
                Transform.pointToWorldFrame(hfPos, hfQuat, hfShape.pillarOffset, worldPillarOffset);
                if (spherePos.distanceTo(worldPillarOffset) < hfShape.pillarConvex.boundingSphereRadius
                        + sphereShape.boundingSphereRadius) {
                    intersecting = this.sphereConvex(
                            sphereShape,
                            hfShape.pillarConvex,
                            spherePos,
                            worldPillarOffset,
                            sphereQuat,
                            hfQuat,
                            sphereBody,
                            hfBody,
                            rsi,
                            rsj,
                            justTest);
                }

                if (justTest && intersecting) {
                    return true;
                }

                int numContacts = result.size() - numContactsBefore;

                if (numContacts > 2) {
                    return false;
                }
                /*
                 * // Skip all but 1
                 * for (int k = 0; k < numContacts - 1; k++) {
                 * result.remove(result.size() - 1);
                 * }
                 */
            }
        }
        return false;
    }

    public boolean boxHeightfield(Box si, Heightfield sj, Vec3 xi, Vec3 xj, Quaternion qi, Quaternion qj, Body bi,
            Body bj, Shape rsi, Shape rsj, boolean justTest) {
        si.convexPolyhedronRepresentation.material = si.material;
        si.convexPolyhedronRepresentation.collisionResponse = si.collisionResponse;
        return this.convexHeightfield(si.convexPolyhedronRepresentation, sj, xi, xj, qi, qj, bi, bj, rsi, rsj,
                justTest);
    }

    public void convexHeightfield(ConvexPolyhedron convexShape, Heightfield hfShape, Vec3 convexPos, Vec3 hfPos,Quaternion convexQuat, Quaternion hfQuat, Body convexBody, Body hfBody, Shape rsi, Shape rsj,boolean justTest) {
        double[][] data = hfShape.data;
        double w = hfShape.elementSize;
        double radius = convexShape.boundingSphereRadius;
        Vec3 worldPillarOffset = convexHeightfield_tmp2;
        // List<Face>
        // added int[] facelist
        //int[] faceList = convexHeightfield_faceList;

        // Get sphere position to heightfield local!
        Vec3 localConvexPos = convexHeightfield_tmp1;
        Transform.pointToLocalFrame(hfPos, hfQuat, convexPos, localConvexPos);

        // Get the index of the data points to test against
        int iMinX = (int) Math.floor((localConvexPos.x - radius) / w) - 1;
        int iMaxX = (int) Math.ceil((localConvexPos.x + radius) / w) + 1;
        int iMinY = (int) Math.floor((localConvexPos.y - radius) / w) - 1;
        int iMaxY = (int) Math.ceil((localConvexPos.y + radius) / w) + 1;

        // Bail out if we are out of the terrain
        if (iMaxX < 0 || iMaxY < 0 || iMinX >= data.length || iMinY >= data[0].length) {
            return;
        }

        // Clamp index to edges
        if (iMinX < 0) {
            iMinX = 0;
        }
        if (iMaxX < 0) {
            iMaxX = 0;
        }
        if (iMinY < 0) {
            iMinY = 0;
        }
        if (iMaxY < 0) {
            iMaxY = 0;
        }
        if (iMinX >= data.length) {
            iMinX = data.length - 1;
        }
        if (iMaxX >= data.length) {
            iMaxX = data.length - 1;
        }
        if (iMaxY >= data[0].length) {
            iMaxY = data[0].length - 1;
        }
        if (iMinY >= data[0].length) {
            iMinY = data[0].length - 1;
        }

        double[] minMax = new double[2];
        hfShape.getRectMinMax(iMinX, iMinY, iMaxX, iMaxY, minMax);
        double min = minMax[0];
        double max = minMax[1];

        // Bail out if we can't touch the bounding height box
        if (localConvexPos.z - radius > max || localConvexPos.z + radius < min) {
            return;
        }

        for (int i = iMinX; i < iMaxX; i++) {
            for (int j = iMinY; j < iMaxY; j++) {
                boolean intersecting = false;

                // Lower triangle
                hfShape.getConvexTrianglePillar(i, j, false);
                Transform.pointToWorldFrame(hfPos, hfQuat, hfShape.pillarOffset, worldPillarOffset);
                if (convexPos.distanceTo(worldPillarOffset) < hfShape.pillarConvex.boundingSphereRadius
                        + convexShape.boundingSphereRadius) {
                    intersecting = this.convexConvex(convexShape, hfShape.pillarConvex, convexPos, worldPillarOffset,
                            convexQuat, hfQuat, convexBody, hfBody, rsi, rsj, justTest, /*added*/new ArrayList<Integer>(), null);
                }

                if (justTest && intersecting) {
                    return;
                }

                // Upper triangle
                hfShape.getConvexTrianglePillar(i, j, true);
                Transform.pointToWorldFrame(hfPos, hfQuat, hfShape.pillarOffset, worldPillarOffset);
                if (convexPos.distanceTo(worldPillarOffset) < hfShape.pillarConvex.boundingSphereRadius
                        + convexShape.boundingSphereRadius) {
                    intersecting = this.convexConvex(convexShape, hfShape.pillarConvex, convexPos, worldPillarOffset,
                            convexQuat, hfQuat, convexBody, hfBody, rsi, rsj, justTest, /*added*/new ArrayList<Integer>(), null);
                }

                if (justTest && intersecting) {
                    return;
                }
            }
        }
    }

    public void sphereParticle(Sphere sj, Particle si, Vec3 xj, Vec3 xi, Quaternion qj, Quaternion qi, Body bj, Body bi,
            Shape rsi, Shape rsj, boolean justTest) {
        // The normal is the unit vector from sphere center to particle center
        Vec3 normal = particleSphere_normal;
        normal.set(0, 0, 1);
        xi.vsub(xj, normal);
        double lengthSquared = normal.lengthSquared();

        if (lengthSquared <= sj.radius * sj.radius) {
            if (justTest) {
                return;
            }
            ContactEquation r = this.createContactEquation(bi, bj, si, sj, rsi, rsj);
            normal.normalize();
            r.rj.copy(normal);
            r.rj.scale(sj.radius, r.rj);
            r.ni.copy(normal); // Contact normal
            r.ni.negate(r.ni);
            r.ri.set(0, 0, 0); // Center of particle
            // changed push to add
            this.result.add(r);
            this.createFrictionEquationsFromContact(r, this.frictionResult);
        }
    }

    public void planeParticle(Plane sj, Particle si, Vec3 xj, Vec3 xi, Quaternion qj, Quaternion qi, Body bj, Body bi,
            Shape rsi, Shape rsj, boolean justTest) {
        Vec3 normal = particlePlane_normal;
        normal.set(0, 0, 1);
        bj.quaternion.vmult(normal, normal); // Turn normal according to plane orientation
        Vec3 relpos = particlePlane_relpos;
        xi.vsub(bj.position, relpos);
        double dot = normal.dot(relpos);

        if (dot <= 0.0) {
            if (justTest) {
                return;
            }

            ContactEquation r = this.createContactEquation(bi, bj, si, sj, rsi, rsj);
            r.ni.copy(normal); // Contact normal is the plane normal
            r.ni.negate(r.ni);
            r.ri.set(0, 0, 0); // Center of particle

            // Get particle position projected on plane
            Vec3 projected = particlePlane_projected;
            normal.scale(normal.dot(xi), projected);
            xi.vsub(projected, projected);

            // rj is now the projected world position minus plane position
            r.rj.copy(projected);
            // changed push to add
            this.result.add(r);
            this.createFrictionEquationsFromContact(r, this.frictionResult);
        }
    }

    public void boxParticle(Box si, Particle sj, Vec3 xi, Vec3 xj, Quaternion qi, Quaternion qj, Body bi, Body bj,
            Shape rsi, Shape rsj, boolean justTest) {
        si.convexPolyhedronRepresentation.material = si.material;
        si.convexPolyhedronRepresentation.collisionResponse = si.collisionResponse;
        this.convexParticle(si.convexPolyhedronRepresentation, sj, xi, xj, qi, qj, bi, bj, si, rsj, justTest);
    }

    public void convexParticle(ConvexPolyhedron sj, Particle si, Vec3 xj, Vec3 xi, Quaternion qj, Quaternion qi,Body bj, Body bi, Shape rsi, Shape rsj, boolean justTest) {
        int penetratedFaceIndex = -1;
        Vec3 penetratedFaceNormal = convexParticle_penetratedFaceNormal;
        Vec3 worldPenetrationVec = convexParticle_worldPenetrationVec;
        Double minPenetration = null;
        int numDetectedFaces = 0;

        // Convert particle position xi to local coords in the convex
        Vec3 local = convexParticle_local;
        local.copy(xi);
        local.vsub(xj, local); // Convert position to relative the convex origin
        qj.conjugate(cqj);
        cqj.vmult(local, local);

        if (sj.pointIsInside(local)) {
            if (sj.worldVerticesNeedsUpdate) {
                sj.computeWorldVertices(xj, qj);
            }
            if (sj.worldFaceNormalsNeedsUpdate) {
                sj.computeWorldFaceNormals(qj);
            }

            // For each world polygon in the polyhedra
            for (int i = 0, nfaces = sj.faces.size(); i != nfaces; i++) {
                // Construct world face vertices
                List<Vec3> verts = new ArrayList<>();
                // changed
                verts.add(sj.worldVertices.get(sj.faces.get(i)[0]));
                Vec3 normal = sj.worldFaceNormals.get(i);

                // Check how much the particle penetrates the polygon plane.
                xi.vsub(verts.get(0), convexParticle_vertexToParticle);
                double penetration = -normal.dot(convexParticle_vertexToParticle);
                if (minPenetration == null || Math.abs(penetration) < Math.abs(minPenetration)) {
                    if (justTest) {
                        return;
                    }

                    minPenetration = penetration;
                    penetratedFaceIndex = i;
                    penetratedFaceNormal.copy(normal);
                    numDetectedFaces++;
                }
            }

            if (penetratedFaceIndex != -1) {
                // Setup contact
                ContactEquation r = this.createContactEquation(bi, bj, si, sj, rsi, rsj);
                penetratedFaceNormal.scale(minPenetration, worldPenetrationVec);

                // rj is the particle position projected to the face
                worldPenetrationVec.vadd(xi, worldPenetrationVec);
                worldPenetrationVec.vsub(xj, worldPenetrationVec);
                r.rj.copy(worldPenetrationVec);
                // const projectedToFace = xi.vsub(xj).vadd(worldPenetrationVec);
                // projectedToFace.copy(r.rj);

                // qj.vmult(r.rj,r.rj);
                penetratedFaceNormal.negate(r.ni); // Contact normal
                r.ri.set(0, 0, 0); // Center of particle

                Vec3 ri = r.ri;
                Vec3 rj = r.rj;

                // Make relative to bodies
                ri.vadd(xi, ri);
                ri.vsub(bi.position, ri);
                rj.vadd(xj, rj);
                rj.vsub(bj.position, rj);

                // changed
                this.result.add(r);
                this.createFrictionEquationsFromContact(r, this.frictionResult);
            } else {
                System.out.println("Point found inside convex, but did not find penetrating face!");
            }
        }
    }

    public void heightfieldCylinder(Heightfield hfShape, Cylinder convexShape, Vec3 hfPos, Vec3 convexPos,
            Quaternion hfQuat, Quaternion convexQuat, Body hfBody, Body convexBody, Shape rsi, Shape rsj,boolean justTest) {
        this.convexHeightfield((ConvexPolyhedron) convexShape, hfShape, convexPos, hfPos, convexQuat, hfQuat,
                convexBody, hfBody, rsi, rsj, justTest);
    }

    public void particleCylinder(Particle si, Cylinder sj, Vec3 xi, Vec3 xj, Quaternion qi, Quaternion qj, Body bi,Body bj, Shape rsi, Shape rsj, boolean justTest) {
        this.convexParticle((ConvexPolyhedron) sj, si, xj, xi, qj, qi, bj, bi, rsi, rsj, justTest);
    }

    public void sphereTrimesh(Sphere sphereShape, Trimesh trimeshShape, Vec3 spherePos, Vec3 trimeshPos,
            Quaternion sphereQuat, Quaternion trimeshQuat, Body sphereBody, Body trimeshBody, Shape rsi, Shape rsj,
            boolean justTest) {
        Vec3 edgeVertexA = sphereTrimesh_edgeVertexA;
        Vec3 edgeVertexB = sphereTrimesh_edgeVertexB;
        Vec3 edgeVector = sphereTrimesh_edgeVector;
        Vec3 edgeVectorUnit = sphereTrimesh_edgeVectorUnit;
        Vec3 localSpherePos = sphereTrimesh_localSpherePos;
        Vec3 tmp = sphereTrimesh_tmp;
        AABB localSphereAABB = sphereTrimesh_localSphereAABB;
        Vec3 v2 = sphereTrimesh_v2;
        Vec3 relpos = sphereTrimesh_relpos;
        List<Integer> triangles = sphereTrimesh_triangles;

        // Convert sphere position to local in the trimesh
        Transform.pointToLocalFrame(trimeshPos, trimeshQuat, spherePos, localSpherePos);

        // Get the aabb of the sphere locally in the trimesh
        double sphereRadius = sphereShape.radius;
        localSphereAABB.lowerBound.set(
                localSpherePos.x - sphereRadius,
                localSpherePos.y - sphereRadius,
                localSpherePos.z - sphereRadius);
        localSphereAABB.upperBound.set(
                localSpherePos.x + sphereRadius,
                localSpherePos.y + sphereRadius,
                localSpherePos.z + sphereRadius);

        trimeshShape.getTrianglesInAABB(localSphereAABB, triangles);

        // Vertices
        Vec3 v = sphereTrimesh_v;
        double radiusSquared = sphereShape.radius * sphereShape.radius;
        for (int i = 0; i < triangles.size(); i++) {
            for (int j = 0; j < 3; j++) {
                trimeshShape.getVertex(trimeshShape.indices[triangles.get(i) * 3 + j], v);

                // Check vertex overlap in sphere
                v.vsub(localSpherePos, relpos);

                if (relpos.lengthSquared() <= radiusSquared) {
                    // Safe up
                    v2.copy(v);
                    Transform.pointToWorldFrame(trimeshPos, trimeshQuat, v2, v);

                    v.vsub(spherePos, relpos);

                    if (justTest) {
                        return;
                    }

                    ContactEquation r = this.createContactEquation(sphereBody, trimeshBody, sphereShape, trimeshShape,
                            rsi, rsj);
                    r.ni.copy(relpos);
                    r.ni.normalize();

                    // ri is the vector from sphere center to the sphere surface
                    r.ri.copy(r.ni);
                    r.ri.scale(sphereShape.radius, r.ri);
                    r.ri.vadd(spherePos, r.ri);
                    r.ri.vsub(sphereBody.position, r.ri);

                    r.rj.copy(v);
                    r.rj.vsub(trimeshBody.position, r.rj);

                    // Store result
                    this.result.add(r);
                    this.createFrictionEquationsFromContact(r, this.frictionResult);
                }
            }
        }

        // Check all edges
        for (int i = 0; i < triangles.size(); i++) {
            for (int j = 0; j < 3; j++) {
                trimeshShape.getVertex(trimeshShape.indices[triangles.get(i) * 3 + j], edgeVertexA);
                trimeshShape.getVertex(trimeshShape.indices[triangles.get(i) * 3 + ((j + 1) % 3)], edgeVertexB);
                edgeVertexB.vsub(edgeVertexA, edgeVector);

                // Project sphere position to the edge
                localSpherePos.vsub(edgeVertexB, tmp);
                double positionAlongEdgeB = tmp.dot(edgeVector);

                localSpherePos.vsub(edgeVertexA, tmp);
                double positionAlongEdgeA = tmp.dot(edgeVector);

                if (positionAlongEdgeA > 0 && positionAlongEdgeB < 0) {
                    // Now check the orthogonal distance from edge to sphere center
                    localSpherePos.vsub(edgeVertexA, tmp);

                    edgeVectorUnit.copy(edgeVector);
                    edgeVectorUnit.normalize();
                    positionAlongEdgeA = tmp.dot(edgeVectorUnit);

                    edgeVectorUnit.scale(positionAlongEdgeA, tmp);
                    tmp.vadd(edgeVertexA, tmp);

                    // tmp is now the sphere center position projected to the edge, defined locally
                    // in the trimesh frame
                    double dist = tmp.distanceTo(localSpherePos);
                    if (dist < sphereShape.radius) {
                        if (justTest) {
                            return;
                        }

                        ContactEquation r = this.createContactEquation(sphereBody, trimeshBody, sphereShape,
                                trimeshShape, rsi, rsj);

                        tmp.vsub(localSpherePos, r.ni);
                        r.ni.normalize();
                        r.ni.scale(sphereShape.radius, r.ri);
                        r.ri.vadd(spherePos, r.ri);
                        r.ri.vsub(sphereBody.position, r.ri);

                        Transform.pointToWorldFrame(trimeshPos, trimeshQuat, tmp, tmp);
                        tmp.vsub(trimeshBody.position, r.rj);

                        Transform.vectorToWorldFrame(trimeshQuat, r.ni, r.ni);
                        Transform.vectorToWorldFrame(trimeshQuat, r.ri, r.ri);

                        this.result.add(r);
                        this.createFrictionEquationsFromContact(r, this.frictionResult);
                    }
                }
            }
        }

        // Triangle faces
        Vec3 va = sphereTrimesh_va;
        Vec3 vb = sphereTrimesh_vb;
        Vec3 vc = sphereTrimesh_vc;
        Vec3 normal = sphereTrimesh_normal;
        for (int i = 0; i < triangles.size(); i++) {
            trimeshShape.getTriangleVertices(triangles.get(i), va, vb, vc);
            trimeshShape.getNormal(triangles.get(i), normal);
            localSpherePos.vsub(va, tmp);
            double dist = tmp.dot(normal);
            normal.scale(dist, tmp);
            localSpherePos.vsub(tmp, tmp);

            // tmp is now the sphere position projected to the triangle plane
            dist = tmp.distanceTo(localSpherePos);
            if (Ray.pointInTriangle(tmp, va, vb, vc) && dist < sphereShape.radius) {
                if (justTest) {
                    return;
                }
                ContactEquation r = this.createContactEquation(sphereBody, trimeshBody, sphereShape, trimeshShape, rsi,
                        rsj);

                tmp.vsub(localSpherePos, r.ni);
                r.ni.normalize();
                r.ni.scale(sphereShape.radius, r.ri);
                r.ri.vadd(spherePos, r.ri);
                r.ri.vsub(sphereBody.position, r.ri);

                Transform.pointToWorldFrame(trimeshPos, trimeshQuat, tmp, tmp);
                tmp.vsub(trimeshBody.position, r.rj);

                Transform.vectorToWorldFrame(trimeshQuat, r.ni, r.ni);
                Transform.vectorToWorldFrame(trimeshQuat, r.ri, r.ri);

                this.result.add(r);
                this.createFrictionEquationsFromContact(r, this.frictionResult);
            }
        }

        triangles.clear();
    }

    public void planeTrimesh(Plane planeShape, Trimesh trimeshShape, Vec3 planePos, Vec3 trimeshPos,Quaternion planeQuat, Quaternion trimeshQuat, Body planeBody, Body trimeshBody, Shape rsi, Shape rsj,boolean justTest) {
        // Make contacts!
        Vec3 v = new Vec3();

        Vec3 normal = planeTrimesh_normal;
        normal.set(0, 0, 1);
        planeQuat.vmult(normal, normal); // Turn normal according to plane

        for (int i = 0; i < trimeshShape.vertices.length / 3; i++) {
            // Get world vertex from trimesh
            trimeshShape.getVertex(i, v);

            // Safe up
            Vec3 v2 = new Vec3();
            v2.copy(v);
            Transform.pointToWorldFrame(trimeshPos, trimeshQuat, v2, v);

            // Check plane side
            Vec3 relpos = planeTrimesh_relpos;
            v.vsub(planePos, relpos);
            double dot = normal.dot(relpos);

            if (dot <= 0.0) {
                if (justTest) {
                    return;
                }

                ContactEquation r = this.createContactEquation(planeBody, trimeshBody, planeShape, trimeshShape, rsi,
                        rsj);

                r.ni.copy(normal); // Contact normal is the plane normal

                // Get vertex position projected on plane
                Vec3 projected = planeTrimesh_projected;
                normal.scale(relpos.dot(normal), projected);
                v.vsub(projected, projected);

                // ri is the projected world position minus plane position
                r.ri.copy(projected);
                r.ri.vsub(planeBody.position, r.ri);

                r.rj.copy(v);
                r.rj.vsub(trimeshBody.position, r.rj);

                // Store result
                this.result.add(r);
                this.createFrictionEquationsFromContact(r, this.frictionResult);
            }
        }
    }

    private Vec3 averageNormal = new Vec3();
    private Vec3 averageContactPointA = new Vec3();
    private Vec3 averageContactPointB = new Vec3();

    private Vec3 tmpVec1 = new Vec3();
    private Vec3 tmpVec2 = new Vec3();
    private Quaternion tmpQuat1 = new Quaternion();
    private Quaternion tmpQuat2 = new Quaternion();

    //private int numWarnings = 0;
    //private final int maxWarnings = 10;

    // private void warn(String msg) {
    //     if (numWarnings > maxWarnings) {
    //         return;
    //     }
    //     numWarnings++;
    //     System.out.println(msg);
    // }

    private Vec3 planeTrimesh_normal = new Vec3();
    private Vec3 planeTrimesh_relpos = new Vec3();
    private Vec3 planeTrimesh_projected = new Vec3();

    private Vec3 sphereTrimesh_normal = new Vec3();
    private Vec3 sphereTrimesh_relpos = new Vec3();
    //private Vec3 sphereTrimesh_projected = new Vec3();
    private Vec3 sphereTrimesh_v = new Vec3();
    private Vec3 sphereTrimesh_v2 = new Vec3();
    private Vec3 sphereTrimesh_edgeVertexA = new Vec3();
    private Vec3 sphereTrimesh_edgeVertexB = new Vec3();
    private Vec3 sphereTrimesh_edgeVector = new Vec3();
    private Vec3 sphereTrimesh_edgeVectorUnit = new Vec3();
    private Vec3 sphereTrimesh_localSpherePos = new Vec3();
    private Vec3 sphereTrimesh_tmp = new Vec3();
    private Vec3 sphereTrimesh_va = new Vec3();
    private Vec3 sphereTrimesh_vb = new Vec3();
    private Vec3 sphereTrimesh_vc = new Vec3();
    private AABB sphereTrimesh_localSphereAABB = new AABB();
    private List<Integer> sphereTrimesh_triangles = new ArrayList<>();

    private Vec3 point_on_plane_to_sphere = new Vec3();
    private Vec3 plane_to_sphere_ortho = new Vec3();

    private Vec3 pointInPolygon_edge = new Vec3();
    private Vec3 pointInPolygon_edge_x_normal = new Vec3();
    private Vec3 pointInPolygon_vtp = new Vec3();

    private boolean pointInPolygon(List<Vec3> verts, Vec3 normal, Vec3 p) {
        Boolean positiveResult = null;
        int N = verts.size();
        for (int i = 0; i < N; i++) {
            Vec3 v = verts.get(i);

            // Get edge to the next vertex
            Vec3 edge = pointInPolygon_edge;
            verts.get((i + 1) % N).vsub(v, edge);

            // Get cross product between polygon normal and the edge
            Vec3 edge_x_normal = pointInPolygon_edge_x_normal;
            edge.cross(normal, edge_x_normal);

            // Get vector between point and current vertex
            Vec3 vertex_to_p = pointInPolygon_vtp;
            p.vsub(v, vertex_to_p);

            // This dot product determines which side of the edge the point is
            double r = edge_x_normal.dot(vertex_to_p);

            // If all such dot products have the same sign, we are inside the polygon.
            if (positiveResult == null || (r > 0 && positiveResult) || (r <= 0 && !positiveResult)) {
                if (positiveResult == null) {
                    positiveResult = r > 0;
                }
                continue;
            } else {
                return false; // Encountered some other sign. Exit.
            }
        }

        // If we got here, all dot products were of the same sign.
        return true;
    }

    private Vec3 box_to_sphere = new Vec3();
    private Vec3 sphereBox_ns = new Vec3();
    private Vec3 sphereBox_ns1 = new Vec3();
    private Vec3 sphereBox_ns2 = new Vec3();
    // changed
    private Vec3[] sphereBox_sides = { new Vec3(), new Vec3(), new Vec3(), new Vec3(), new Vec3(), new Vec3() };
    private Vec3 sphereBox_sphere_to_corner = new Vec3();
    private Vec3 sphereBox_side_ns = new Vec3();
    private Vec3 sphereBox_side_ns1 = new Vec3();
    private Vec3 sphereBox_side_ns2 = new Vec3();

    private Vec3 convex_to_sphere = new Vec3();
    private Vec3 sphereConvex_edge = new Vec3();
    private Vec3 sphereConvex_edgeUnit = new Vec3();
    private Vec3 sphereConvex_sphereToCorner = new Vec3();
    private Vec3 sphereConvex_worldCorner = new Vec3();
    private Vec3 sphereConvex_worldNormal = new Vec3();
    private Vec3 sphereConvex_worldPoint = new Vec3();
    private Vec3 sphereConvex_worldSpherePointClosestToPlane = new Vec3();
    private Vec3 sphereConvex_penetrationVec = new Vec3();
    private Vec3 sphereConvex_sphereToWorldPoint = new Vec3();

    //private Vec3 planeBox_normal = new Vec3();
    //private Vec3 plane_to_corner = new Vec3();

    private Vec3 planeConvex_v = new Vec3();
    private Vec3 planeConvex_normal = new Vec3();
    private Vec3 planeConvex_relpos = new Vec3();
    private Vec3 planeConvex_projected = new Vec3();

    private Vec3 convexConvex_sepAxis = new Vec3();
    private Vec3 convexConvex_q = new Vec3();

    private Vec3 particlePlane_normal = new Vec3();
    private Vec3 particlePlane_relpos = new Vec3();
    private Vec3 particlePlane_projected = new Vec3();

    private Vec3 particleSphere_normal = new Vec3();

    // WIP
    private Quaternion cqj = new Quaternion();
    private Vec3 convexParticle_local = new Vec3();
    //private Vec3 convexParticle_normal = new Vec3();
    private Vec3 convexParticle_penetratedFaceNormal = new Vec3();
    private Vec3 convexParticle_vertexToParticle = new Vec3();
    private Vec3 convexParticle_worldPenetrationVec = new Vec3();

    private Vec3 convexHeightfield_tmp1 = new Vec3();
    private Vec3 convexHeightfield_tmp2 = new Vec3();
    //private int[] convexHeightfield_faceList = { 0 };

    private Vec3 sphereHeightfield_tmp1 = new Vec3();
    private Vec3 sphereHeightfield_tmp2 = new Vec3();

}
