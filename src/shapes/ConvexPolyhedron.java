package shapes;

import java.util.ArrayList;
import java.util.List;

import math.Vec3;
import math.Quaternion;
import math.Transform;

/**
 * A set of polygons describing a convex shape.
 *
 * The shape MUST be convex for the code to work properly. No polygons may be
 * coplanar (contained
 * in the same 3D plane), instead these should be merged into one polygon.
 *
 * @author qiao / https://github.com/qiao (original author, see
 *         https://github.com/qiao/three.js/commit/85026f0c769e4000148a67d45a9e9b9c5108836f)
 * @author schteppe / https://github.com/schteppe
 * @see https://www.altdevblogaday.com/2011/05/13/contact-generation-between-3d-convex-meshes/
 *
 * @todo Move the clipping functions to ContactGenerator?
 * @todo Automatically merge coplanar polygons in constructor.
 * @example
 *          const convexShape = new CANNON.ConvexPolyhedron({ vertices, faces })
 *          const convexBody = new CANNON.Body({ mass: 1, shape: convexShape })
 *          world.addBody(convexBody)
 */

public class ConvexPolyhedron extends Shape {
    public List<Vec3> vertices;
    /**
     * Array of integer arrays, indicating which vertices each face consists of
     */
    public List<int[]> faces;
    public List<Vec3> faceNormals;
    public List<Vec3> worldVertices;
    public boolean worldVerticesNeedsUpdate;
    public List<Vec3> worldFaceNormals;
    public boolean worldFaceNormalsNeedsUpdate;
    /**
     * If given, these locally defined, normalized axes are the only ones being
     * checked when doing separating axis check.
     */
    private List<Vec3> uniqueAxes;
    private List<Vec3> uniqueEdges;

    public ConvexPolyhedron() {
        super(ShapeTypes.CONVEXPOLYHEDRON);// Shape.types to Shape
        vertices = new ArrayList<>();
        faces = new ArrayList<>();
        faceNormals = new ArrayList<>();
        worldVertices = new ArrayList<>();
        worldVerticesNeedsUpdate = true;
        worldFaceNormals = new ArrayList<>();
        worldFaceNormalsNeedsUpdate = true;
        uniqueAxes = null;
        uniqueEdges = new ArrayList<>();
        computeEdges();
    }

    /**
     * @param vertices An array of Vec3's
     * @param faces    Array of integer arrays, describing which vertices that is
     *                 included in each face.
     */
    public ConvexPolyhedron(List<Vec3> vertices, List<int[]> faces, List<Vec3> normals, List<Vec3> axes,double boundingSphereRadius) {
        super(ShapeTypes.CONVEXPOLYHEDRON);// Shape.types to Shape
        this.vertices = vertices;
        this.faces = faces;
        this.faceNormals = normals;

        if (this.faceNormals.isEmpty()) {
            computeNormals();
        }

        if (boundingSphereRadius == 0.0) {
            updateBoundingSphereRadius();
        } else {
            this.boundingSphereRadius = boundingSphereRadius;
        }

        worldVertices = new ArrayList<>();
        worldVerticesNeedsUpdate = true;
        worldFaceNormals = new ArrayList<>();
        worldFaceNormalsNeedsUpdate = true;
        uniqueAxes = axes != null ? new ArrayList<>(axes) : null;
        uniqueEdges = new ArrayList<>();
        computeEdges();
    }

    /**
     * Computes uniqueEdges
     */
    public void computeEdges() {
        List<int[]> faces = this.faces;
        List<Vec3> vertices = this.vertices;
        List<Vec3> edges = uniqueEdges;
        edges.clear();
        Vec3 edge = new Vec3();

        for (int i = 0; i < faces.size(); i++) {
            int[] face = faces.get(i);
            int numVertices = face.length;

            for (int j = 0; j < numVertices; j++) {
                int k = (j + 1) % numVertices;
                vertices.get(face[j]).vsub(vertices.get(face[k]), edge);
                edge.normalize();
                boolean found = false;

                for (int p = 0; p < edges.size(); p++) {
                    if (edges.get(p).almostEquals(edge) || edges.get(p).almostEquals(edge)) {
                        found = true;
                        break;
                    }
                }

                if (!found) {
                    edges.add(edge.clone());
                }
            }
        }
    }

    /**
     * Compute the normals of the faces.
     * Will reuse existing Vec3 objects in the `faceNormals` array if they exist.
     */
    public void computeNormals() {
        faceNormals.clear();

        // Generate normals
        for (int i = 0; i < faces.size(); i++) {
            // added
            for (int j = 0; j < this.faces.get(i).length; j++) {
                if (this.vertices.get(this.faces.get(i)[j]) != null)
                    System.err.println("Vertex" + this.faces.get(i)[j] + "not found!");
            }

            int[] face = faces.get(i);
            int numVertices = face.length;
            Vec3 n = faceNormals.size() > i ? faceNormals.get(i) : new Vec3();
            getFaceNormal(i, n);
            n.negate(n);
            faceNormals.set(i, n);
            Vec3 vertex = vertices.get(face[0]);

            if (n.dot(vertex) < 0) {
                System.err.println(".faceNormals[" + i + "] = Vec3(" + n.toString()
                        + ") looks like it points into the shape? The vertices follow. Make sure they are ordered CCW around the normal, using the right hand rule.");

                for (int j = 0; j < numVertices; j++) {
                    System.out.println(".vertices[" + face[j] + "] = Vec3(" + vertices.get(face[j]).toString() + ")");
                }
            }
        }
    }

    /**
     * Compute the normal of a face from its vertices
     */
    public void getFaceNormal(int i, Vec3 target) {
        int[] f = faces.get(i);
        Vec3 va = vertices.get(f[0]);
        Vec3 vb = vertices.get(f[1]);
        Vec3 vc = vertices.get(f[2]);
        ConvexPolyhedron.computeNormal(va, vb, vc, target);
    }

    /**
     * Get face normal given 3 vertices
     */
    public static void computeNormal(Vec3 va, Vec3 vb, Vec3 vc, Vec3 target) {
        Vec3 cb = new Vec3();
        Vec3 ab = new Vec3();
        vb.vsub(va, ab);
        vc.vsub(vb, cb);
        cb.cross(ab, target);

        if (!target.isZero()) {
            target.normalize();
        }
    }

    /**
     * @param minDist Clamp distance
     * @param result  The an array of contact point objects, see clipFaceAgainstHull
     */
    public void clipAgainstHull(Vec3 posA, Quaternion quatA, ConvexPolyhedron hullB, Vec3 posB, Quaternion quatB,Vec3 separatingNormal, double minDist, double maxDist, List<ConvexPolyhedronContactPoint> result) {
        Vec3 worldNormal = new Vec3();
        int closestFaceB = -1;
        double dmax = -Double.MAX_VALUE;

        for (int face = 0; face < hullB.faces.size(); face++) {
            worldNormal.copy(hullB.faceNormals.get(face));
            quatB.vmult(worldNormal, worldNormal);
            double d = worldNormal.dot(separatingNormal);

            if (d > dmax) {
                dmax = d;
                closestFaceB = face;
            }
        }

        // added second for loop

        List<Vec3> worldVertsB1 = new ArrayList<Vec3>();

        for (int i = 0; i < hullB.faces.get(closestFaceB).length; i++) {
            Vec3 b = hullB.vertices.get(hullB.faces.get(closestFaceB)[i]);
            Vec3 worldb = b;
            quatB.vmult(worldb, worldb);
            posB.vadd(worldb, worldb);
            worldVertsB1.add(worldb);
        }

        if (closestFaceB >= 0) {
            this.clipFaceAgainstHull(separatingNormal, posA, quatA, worldVertsB1, minDist, maxDist, result);
        }

        // if (closestFaceB >= 0) {
        // // Transformed vertices of hullB
        // List<Vec3> worldVerticesB = hullB.worldVertices;
        // List<Vec3> closestFaceBVertices = new ArrayList<>();

        // for (int i = 0; i < hullB.faces.get(closestFaceB).length; i++) {
        // int vertexIndexB = hullB.faces.get(closestFaceB)[i];
        // Vec3 worldVertexB = worldVerticesB.get(vertexIndexB);
        // closestFaceBVertices.add(worldVertexB.clone());
        // }

        // // Vertices of the clipping hull
        // List<Vec3> worldVerticesA = worldVertices;
        // List<Vec3> hullAEdges = uniqueEdges;
        // List<Vec3> clippedVertices = closestFaceBVertices;

        // // Add the separating axis
        // clippedVertices.add(separatingNormal.clone());

        // // Clip the hull by the separating axis
        // ConvexPolyhedron.clipFaceAgainstHull(separatingNormal.negate(),
        // worldVerticesA, hullAEdges, posA, quatA, clippedVertices);

        // // Find the closest points on the clipped face
        // Vec3 c1 = new Vec3(); // contact point on hull A
        // Vec3 c2 = new Vec3(); // contact point on hull B
        // double minDist = Double.MAX_VALUE;

        // for (int i = 0; i < clippedVertices.size(); i++) {
        // double vertexDist = clippedVertices.get(i).dot(separatingNormal);

        // if (vertexDist < minDist) {
        // minDist = vertexDist;
        // c1.copy(clippedVertices.get(i));
        // c2.copy(clippedVertices.get(i));
        // }
        // }

        // // Calculate contact point information
        // c1.vsub(posA, c1);
        // c2.vsub(posB, c2);
        // double dist = c1.distanceTo(c2);

        // if (dist < maxDist && dist >= minDist) {
        // ConvexPolyhedronContactPoint pt = new ConvexPolyhedronContactPoint();
        // pt.pointInWorld = c2.clone();
        // pt.normalWorld = separatingNormal.clone();
        // pt.depth = dist;
        // result.add(pt);
        // }
        // }

    }

    /**
     * Find the separating axis between this hull and another
     * 
     * @param target The target vector to save the axis in
     * @return Returns false if a separation is found, else true
     */
    public boolean findSeparatingAxis(ConvexPolyhedron hullB, Vec3 posA, Quaternion quatA, Vec3 posB, Quaternion quatB,
            Vec3 target, List<Integer> faceListA, List<Integer> faceListB) {
        Vec3 faceANormalWS3 = new Vec3();
        Vec3 Worldnormal1 = new Vec3();
        Vec3 deltaC = new Vec3();
        Vec3 worldEdge0 = new Vec3();
        Vec3 worldEdge1 = new Vec3();
        Vec3 Cross = new Vec3();

        double dmin = Double.MAX_VALUE;
        ConvexPolyhedron hullA = this;
        int curPlaneTests = 0;

        if (hullA.uniqueAxes == null) {
            int numFacesA = (faceListA != null) ? faceListA.size() : hullA.faces.size();

            // Test face normals from hullA
            for (int i = 0; i < numFacesA; i++) {
                int fi = (faceListA != null) ? faceListA.get(i) : i;

                // Get world face normal
                faceANormalWS3.copy(hullA.faceNormals.get(fi));
                quatA.vmult(faceANormalWS3, faceANormalWS3);

                double d = hullA.testSepAxis(faceANormalWS3, hullB, posA, quatA, posB, quatB);
                if (d == false) {
                    return false;
                }

                if (d < dmin) {
                    dmin = d;
                    target.copy(faceANormalWS3);
                }
            }
        } else {
            // Test unique axes
            for (int i = 0; i < hullA.uniqueAxes.size(); i++) {
                // Get world axis
                quatA.vmult(hullA.uniqueAxes.get(i), faceANormalWS3);

                double d = hullA.testSepAxis(faceANormalWS3, hullB, posA, quatA, posB, quatB);
                if (d == false) {
                    return false;
                }

                if (d < dmin) {
                    dmin = d;
                    target.copy(faceANormalWS3);
                }
            }
        }

        if (hullB.uniqueAxes == null) {
            // Test face normals from hullB
            int numFacesB = (faceListB != null) ? faceListB.size() : hullB.faces.size();
            for (int i = 0; i < numFacesB; i++) {
                int fi = (faceListB != null) ? faceListB.get(i) : i;

                Worldnormal1.copy(hullB.faceNormals.get(fi));
                quatB.vmult(Worldnormal1, Worldnormal1);
                curPlaneTests++;
                double d = hullA.testSepAxis(Worldnormal1, hullB, posA, quatA, posB, quatB);
                if (d == false) {
                    return false;
                }

                if (d < dmin) {
                    dmin = d;
                    target.copy(Worldnormal1);
                }
            }
        } else {
            // Test unique axes in B
            for (int i = 0; i < hullB.uniqueAxes.size(); i++) {
                quatB.vmult(hullB.uniqueAxes.get(i), Worldnormal1);

                curPlaneTests++;
                double d = hullA.testSepAxis(Worldnormal1, hullB, posA, quatA, posB, quatB);
                if (d == false) {
                    return false;
                }

                if (d < dmin) {
                    dmin = d;
                    target.copy(Worldnormal1);
                }
            }
        }

        // Test edges
        for (int e0 = 0; e0 < hullA.uniqueEdges.size(); e0++) {
            // Get world edge
            quatA.vmult(hullA.uniqueEdges.get(e0), worldEdge0);

            for (int e1 = 0; e1 < hullB.uniqueEdges.size(); e1++) {
                // Get world edge 2
                quatB.vmult(hullB.uniqueEdges.get(e1), worldEdge1);
                worldEdge0.cross(worldEdge1, Cross);

                if (!Cross.almostZero()) {
                    Cross.normalize();
                    double dist = hullA.testSepAxis(Cross, hullB, posA, quatA, posB, quatB);
                    if (dist == false) {
                        return false;
                    }
                    if (dist < dmin) {
                        dmin = dist;
                        target.copy(Cross);
                    }
                }
            }
        }

        posB.vsub(posA, deltaC);
        if (deltaC.dot(target) > 0.0) {
            target.negate(target);
        }

        return true;
    }

    /**
     * Test separating axis against two hulls. Both hulls are projected onto the
     * axis and the overlap size is returned if there is one.
     * 
     * @return The overlap depth, or -1 if no penetration.
     */
    public double testSepAxis(Vec3 axis, ConvexPolyhedron hullB, Vec3 posA, Quaternion quatA, Vec3 posB,
            Quaternion quatB) {
        double[] maxminA = new double[2]; // added
        double[] maxminB = new double[2];
        ConvexPolyhedron.project(this, axis, posA, quatA, maxminA);
        ConvexPolyhedron.project(hullB, axis, posB, quatB, maxminB);
        double maxA = maxminA[0];
        double minA = maxminA[1];
        double maxB = maxminB[0];
        double minB = maxminB[1];
        if (maxA < minB || maxB < minA) {
            //added : return -1 for no penetration
            return -1; // Separated
        }
        double d0 = maxA - minB;
        double d1 = maxB - minA;
        double depth = (d0 < d1) ? d0 : d1;
        return depth;
    }

    public void calculateLocalInertia(double mass, Vec3 target) {
        // Approximate with box inertia
        // Exact inertia calculation is overkill, but see
        // http://geometrictools.com/Documentation/PolyhedralMassProperties.pdf for the
        // correct way to do it
        Vec3 aabbmax = new Vec3();
        Vec3 aabbmin = new Vec3();
        this.computeLocalAABB(aabbmin, aabbmax);
        double x = aabbmax.x - aabbmin.x;
        double y = aabbmax.y - aabbmin.y;
        double z = aabbmax.z - aabbmin.z;
        target.x = (1.0 / 12.0) * mass * (2 * y * 2 * y + 2 * z * 2 * z);
        target.y = (1.0 / 12.0) * mass * (2 * x * 2 * x + 2 * z * 2 * z);
        target.z = (1.0 / 12.0) * mass * (2 * y * 2 * y + 2 * x * 2 * x);

        // return target ; //added return type Vec3 as per super implementation
    }

    /**
     * @param face_i Index of the face
     */
    public double getPlaneConstantOfFace(int face_i) {
        int[] f = this.faces.get(face_i);
        Vec3 n = this.faceNormals.get(face_i);
        Vec3 v = this.vertices.get(f[0]);
        double c = -n.dot(v);
        return c;
    }

    /**
     * Clip a face against a hull.
     * 
     * @param worldVertsB1 An array of Vec3 with vertices in the world frame.
     * @param minDist      Distance clamping
     * @param Array        result Array to store resulting contact points in. Will
     *                     be objects with properties: point, depth, normal. These
     *                     are represented in world coordinates.
     */
    public void clipFaceAgainstHull(Vec3 separatingNormal, Vec3 posA, Quaternion quatA, List<Vec3> worldVertsB1,
            double minDist, double maxDist, List<ConvexPolyhedronContactPoint> result) {
        Vec3 faceANormalWS = new Vec3();
        Vec3 edge0 = new Vec3();
        Vec3 WorldEdge0 = new Vec3();
        Vec3 worldPlaneAnormal1 = new Vec3();
        Vec3 planeNormalWS1 = new Vec3();
        Vec3 worldA1 = new Vec3();
        Vec3 localPlaneNormal = new Vec3();
        Vec3 planeNormalWS = new Vec3();
        ConvexPolyhedron hullA = this;
        List<Vec3> worldVertsB2 = new ArrayList<>();
        List<Vec3> pVtxIn = worldVertsB1;
        List<Vec3> pVtxOut = worldVertsB2;

        int closestFaceA = -1;
        double dmin = Double.MAX_VALUE;

        // Find the face with normal closest to the separating axis
        for (int face = 0; face < hullA.faces.size(); face++) {
            faceANormalWS.copy(hullA.faceNormals.get(face));
            quatA.vmult(faceANormalWS, faceANormalWS);
            double d = faceANormalWS.dot(separatingNormal);
            if (d < dmin) {
                dmin = d;
                closestFaceA = face;
            }
        }
        if (closestFaceA < 0) {
            return;
        }

        // Get the face and construct connected faces
        int[] polyA = hullA.faces.get(closestFaceA);
        polyA.connectedFaces = new ArrayList<>();
        for (int i = 0; i < hullA.faces.size(); i++) {
            for (int j = 0; j < hullA.faces.get(i).length; j++) {
                if (
                /* Sharing a vertex */
                polyA.indexOf(hullA.faces.get(i)[j]) != -1 &&
                /* Not the one we are looking for connections from */
                        i != closestFaceA &&
                        /* Not already added */
                        polyA.connectedFaces.indexOf(i) == -1) {
                    polyA.connectedFaces.add(i);
                }
            }
        }

        // Clip the polygon to the back of the planes of all faces of hull A,
        // that are adjacent to the witness face
        int numVerticesA = polyA.length;
        for (int i = 0; i < numVerticesA; i++) {
            Vec3 a = hullA.vertices.get(polyA[i]);
            Vec3 b = hullA.vertices.get(polyA[(i + 1) % numVerticesA]);
            a.vsub(b, edge0);
            WorldEdge0.copy(edge0);
            quatA.vmult(WorldEdge0, WorldEdge0);
            posA.vadd(WorldEdge0, WorldEdge0);
            worldPlaneAnormal1.copy(this.faceNormals.get(closestFaceA));
            quatA.vmult(worldPlaneAnormal1, worldPlaneAnormal1);
            posA.vadd(worldPlaneAnormal1, worldPlaneAnormal1);
            WorldEdge0.cross(worldPlaneAnormal1, planeNormalWS1);
            planeNormalWS1.negate(planeNormalWS1);
            worldA1.copy(a);
            quatA.vmult(worldA1, worldA1);
            posA.vadd(worldA1, worldA1);

            int otherFace = polyA.connectedFaces.get(i);
            localPlaneNormal.copy(this.faceNormals.get(otherFace));
            double localPlaneEq = this.getPlaneConstantOfFace(otherFace);
            planeNormalWS.copy(localPlaneNormal);
            quatA.vmult(planeNormalWS, planeNormalWS);
            double planeEqWS = localPlaneEq - planeNormalWS.dot(posA);

            // Clip face against our constructed plane
            this.clipFaceAgainstPlane(pVtxIn, pVtxOut, planeNormalWS, planeEqWS);

            // Throw away all clipped points, but save the remaining until next clip
            pVtxIn.clear();
            pVtxIn.addAll(pVtxOut);
            pVtxOut.clear();
        }

        // only keep contact points that are behind the witness face
        localPlaneNormal.copy(this.faceNormals.get(closestFaceA));
        double localPlaneEq = this.getPlaneConstantOfFace(closestFaceA);
        planeNormalWS.copy(localPlaneNormal);
        quatA.vmult(planeNormalWS, planeNormalWS);
        double planeEqWS = localPlaneEq - planeNormalWS.dot(posA);
        for (int i = 0; i < pVtxIn.size(); i++) {
            double depth = planeNormalWS.dot(pVtxIn.get(i)) + planeEqWS;

            if (depth <= minDist) {
                System.out.println("clamped: depth=" + depth + " to minDist=" + minDist);
                depth = minDist;
            }

            if (depth <= maxDist) {
                Vec3 point = pVtxIn.get(i);
                if (depth <= 1e-6) {
                    ConvexPolyhedronContactPoint p = new ConvexPolyhedronContactPoint(point, planeNormalWS, depth);
                    result.add(p);
                }
            }
        }
    }

    /**
   * Clip a face in a hull against the back of a plane.
   * @param planeConstant The constant in the mathematical plane equation
   */
    public List<Vec3> clipFaceAgainstPlane(List<Vec3> inVertices, List<Vec3> outVertices, Vec3 planeNormal,double planeConstant) {
        double n_dot_first;
        double n_dot_last;
        int numVerts = inVertices.size();

        if (numVerts < 2) {
            return outVertices;
        }

        Vec3 firstVertex = inVertices.get(inVertices.size() - 1);
        Vec3 lastVertex = inVertices.get(0);

        n_dot_first = planeNormal.dot(firstVertex) + planeConstant;

        for (int vi = 0; vi < numVerts; vi++) {
            lastVertex = inVertices.get(vi);
            n_dot_last = planeNormal.dot(lastVertex) + planeConstant;
            if (n_dot_first < 0) {
                if (n_dot_last < 0) {
                    // Start < 0, end < 0, so output lastVertex
                    Vec3 newv = new Vec3();
                    newv.copy(lastVertex);
                    outVertices.add(newv);
                } else {
                    // Start < 0, end >= 0, so output intersection
                    Vec3 newv = new Vec3();
                    firstVertex.lerp(lastVertex, n_dot_first / (n_dot_first - n_dot_last), newv);
                    outVertices.add(newv);
                }
            } else {
                if (n_dot_last < 0) {
                    // Start >= 0, end < 0 so output intersection and end
                    Vec3 newv = new Vec3();
                    firstVertex.lerp(lastVertex, n_dot_first / (n_dot_first - n_dot_last), newv);
                    outVertices.add(newv);
                    outVertices.add(lastVertex);
                }
            }
            firstVertex = lastVertex;
            n_dot_first = n_dot_last;
        }
        return outVertices;
    }

    /**
   * Updates `.worldVertices` and sets `.worldVerticesNeedsUpdate` to false.
   */
    public void computeWorldVertices(Vec3 position, Quaternion quat) {
        while (this.worldVertices.size() < this.vertices.size()) {
            this.worldVertices.add(new Vec3());
        }

        List<Vec3> verts = this.vertices;
        List<Vec3> worldVerts = this.worldVertices;
        for (int i = 0; i < this.vertices.size(); i++) {
            // changed , made it same as source cod
            quat.vmult(verts.get(i), worldVerts.get(i));
            position.vadd(worldVerts.get(i), worldVerts.get(i));
            // worldVerts.set(i, vertex);
        }

        this.worldVerticesNeedsUpdate = false;
    }

    public void computeLocalAABB(Vec3 aabbmin, Vec3 aabbmax) {
        List<Vec3> vertices = this.vertices;

        aabbmin.set(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        aabbmax.set(-Double.MAX_VALUE, -Double.MAX_VALUE, -Double.MAX_VALUE);

        for (int i = 0; i < this.vertices.size(); i++) {
            Vec3 v = vertices.get(i);
            if (v.x < aabbmin.x) {
                aabbmin.x = v.x;
            } else if (v.x > aabbmax.x) {
                aabbmax.x = v.x;
            }
            if (v.y < aabbmin.y) {
                aabbmin.y = v.y;
            } else if (v.y > aabbmax.y) {
                aabbmax.y = v.y;
            }
            if (v.z < aabbmin.z) {
                aabbmin.z = v.z;
            } else if (v.z > aabbmax.z) {
                aabbmax.z = v.z;
            }
        }
    }

    /**
   * Updates `worldVertices` and sets `worldVerticesNeedsUpdate` to false.
   */
    public void computeWorldFaceNormals(Quaternion quat) {
        int N = this.faceNormals.size();
        while (this.worldFaceNormals.size() < N) {
            this.worldFaceNormals.add(new Vec3());
        }

        List<Vec3> normals = this.faceNormals;
        List<Vec3> worldNormals = this.worldFaceNormals;
        for (int i = 0; i < N; i++) {
            quat.vmult(normals.get(i), worldNormals.get(i));
        }

        this.worldFaceNormalsNeedsUpdate = false;
    }

     /**
   * updateBoundingSphereRadius
   */
    public void updateBoundingSphereRadius() {
        double max2 = 0;
        List<Vec3> verts = this.vertices;
        for (int i = 0; i < verts.size(); i++) {
            double norm2 = verts.get(i).lengthSquared();
            if (norm2 > max2) {
                max2 = norm2;
            }
        }
        this.boundingSphereRadius = Math.sqrt(max2);
    }

    public void calculateWorldAABB(Vec3 pos, Quaternion quat, Vec3 min, Vec3 max) {
        List<Vec3> verts = this.vertices;
        Double minx = null, miny = null, minz = null, maxx = null, maxy = null, maxz = null;
        Vec3 tempWorldVertex = new Vec3();
        for (int i = 0; i < verts.size(); i++) {
            tempWorldVertex.copy(verts.get(i));
            quat.vmult(tempWorldVertex, tempWorldVertex);
            pos.vadd(tempWorldVertex, tempWorldVertex);
            Vec3 v = tempWorldVertex;
            if (minx == null || v.x < minx) {
                minx = v.x;
            }
            if (maxx == null || v.x > maxx) {
                maxx = v.x;
            }
            if (miny == null || v.y < miny) {
                miny = v.y;
            }
            if (maxy == null || v.y > maxy) {
                maxy = v.y;
            }
            if (minz == null || v.z < minz) {
                minz = v.z;
            }
            if (maxz == null || v.z > maxz) {
                maxz = v.z;
            }
        }
        min.set(minx, miny, minz);
        max.set(maxx, maxy, maxz);
    }

    /**
   * Get approximate convex volume
   */
    public double volume() {
        return (4.0 * Math.PI * this.boundingSphereRadius) / 3.0;
    }

    /**
   * Get an average of all the vertices positions
   */
    public Vec3 getAveragePointLocal(Vec3 target) {
        List<Vec3> verts = this.vertices;
        target.set(0, 0, 0);
        for (int i = 0; i < verts.size(); i++) {
            target.vadd(verts.get(i), target);
        }
        target.scale(1.0 / verts.size(), target);
        return target;
    }

    /**
   * Transform all local points. Will change the .vertices
   */
    public void transformAllPoints(Vec3 offset, Quaternion quat) {
        int n = this.vertices.size();
        List<Vec3> verts = this.vertices;

        // Apply rotation
        if (quat != null) {
            // Rotate vertices
            for (int i = 0; i < n; i++) {
                Vec3 v = verts.get(i);
                quat.vmult(v, v);
            }
            // Rotate face normals
            for (int i = 0; i < this.faceNormals.size(); i++) {
                Vec3 v = this.faceNormals.get(i);
                quat.vmult(v, v);
            }
            /*
            // Rotate edges
            for(let i=0; i<this.uniqueEdges.length; i++){
                const v = this.uniqueEdges[i];
                quat.vmult(v,v);
            }*/
        }

        // Apply offset
        if (offset != null) {
            for (int i = 0; i < n; i++) {
                Vec3 v = verts.get(i);
                v.vadd(offset, v);
            }
        }
    }

    /**
   * Checks whether p is inside the polyhedra. Must be in local coords.
   * The point lies outside of the convex hull of the other points if and only if the direction
   * of all the vectors from it to those other points are on less than one half of a sphere around it.
   * @param p A point given in local coordinates
   */
    public int pointIsInside(Vec3 p) {
        List<Vec3> verts = this.vertices;
        // change List<Vec3> to List<int[]>
        List<int[]> faces = this.faces;
        List<Vec3> normals = this.faceNormals;
        Integer positiveResult = null;
        Vec3 pointInside = new Vec3();
        this.getAveragePointLocal(pointInside);

        for (int i = 0; i < this.faces.size(); i++) {
            Vec3 n = normals.get(i);
            Vec3 v = verts.get(faces.get(i)[0]); // We only need one point in the face

            // This dot product determines which side of the edge the point is
            Vec3 vToP = p.vsub(v);
            double r1 = n.dot(vToP);

            Vec3 vToPointInside = pointInside.vsub(v);
            double r2 = n.dot(vToPointInside);

            if ((r1 < 0 && r2 > 0) || (r1 > 0 && r2 < 0)) {
                return 0 ; //changed return type from false to 0 ; // Encountered some other sign. Exit.
            }
        }

        // If we got here, all dot products were of the same sign.
        return positiveResult != null ? 1 : -1;
    }

    public static void project(ConvexPolyhedron shape, Vec3 axis, Vec3 pos, Quaternion quat, double[] result) {
        int n = shape.vertices.size();
        Vec3 worldVertex = new Vec3();
        Vec3 localAxis = new Vec3();
        double max = 0;
        double min = 0;
        Vec3 localOrigin = new Vec3();
        List<Vec3> vs = shape.vertices;

        localOrigin.setZero();

        // Transform the axis to local
        Transform.vectorToLocalFrame(pos, quat, axis, localAxis);
        Transform.pointToLocalFrame(pos, quat, localOrigin, localOrigin);
        double add = localOrigin.dot(localAxis);

        min = max = vs.get(0).dot(localAxis);

        for (int i = 1; i < n; i++) {
            double val = vs.get(i).dot(localAxis);

            if (val > max) {
                max = val;
            }

            if (val < min) {
                min = val;
            }
        }

        min -= add;
        max -= add;

        if (min > max) {
            // Inconsistent - swap
            double temp = min;
            min = max;
            max = temp;
        }
        // Output
        result[0] = max;
        result[1] = min;
    }

    // constants

    static List<Double> maxminA = new ArrayList<>();
    static List<Double> maxminB = new ArrayList<>();

    static Vec3 project_worldVertex = new Vec3();
    static Vec3 project_localAxis = new Vec3();
    static Vec3 project_localOrigin = new Vec3();
}
