package world;

import shapes.ShapeTypes;

// Naming rule: based of the order in SHAPE_TYPES,
// the first part of the method is formed by the
// shape type that comes before, in the second part
// there is the shape type that comes after in the SHAPE_TYPES list

public class CollisionTypes {
    // Define integer constants for each collision type
    public static final int SPHERE_SPHERE = ShapeTypes.SPHERE.getValue();//1
    public static final int SPHERE_PLANE = ShapeTypes.SPHERE.getValue()| ShapeTypes.PLANE.getValue() ;//| 3;
    public static final int BOX_BOX = ShapeTypes.BOX.getValue() | ShapeTypes.BOX.getValue() ;//| 4;
    public static final int SPHERE_BOX = ShapeTypes.SPHERE.getValue() | ShapeTypes.BOX.getValue();// | 5;
    public static final int PLANE_BOX = ShapeTypes.PLANE.getValue() | ShapeTypes.BOX.getValue();// | 6;
    public static final int CONVEX_CONVEX = ShapeTypes.CONVEXPOLYHEDRON.getValue();// | 16;
    public static final int SPHERE_CONVEX = ShapeTypes.SPHERE.getValue() | ShapeTypes.CONVEXPOLYHEDRON.getValue();// | 17;
    public static final int PLANE_CONVEX = ShapeTypes.PLANE.getValue() | ShapeTypes.CONVEXPOLYHEDRON.getValue();// | 18;
    public static final int BOX_CONVEX = ShapeTypes.BOX.getValue() | ShapeTypes.CONVEXPOLYHEDRON.getValue();//| 20;
    public static final int SPHERE_HEIGHTFIELD = ShapeTypes.SPHERE.getValue() | ShapeTypes.HEIGHTFIELD.getValue();// | 33;
    public static final int BOX_HEIGHTFIELD = ShapeTypes.BOX.getValue() | ShapeTypes.HEIGHTFIELD.getValue();//| 36;
    public static final int CONVEX_HEIGHTFIELD = ShapeTypes.CONVEXPOLYHEDRON.getValue() | ShapeTypes.HEIGHTFIELD.getValue();// | 48;
    public static final int SPHERE_PARTICLE = ShapeTypes.PARTICLE.getValue() | ShapeTypes.SPHERE.getValue() ; // | 65;
    public static final int PLANE_PARTICLE = ShapeTypes.PLANE.getValue() | ShapeTypes.PARTICLE.getValue() ;// | 66;
    public static final int BOX_PARTICLE = ShapeTypes.BOX.getValue() | ShapeTypes.PARTICLE.getValue() ; //| 68;
    public static final int CONVEX_PARTICLE = ShapeTypes.PARTICLE.getValue() | ShapeTypes.CONVEXPOLYHEDRON.getValue() ; // | 80;
    public static final int CYLINDER_CYLINDER = ShapeTypes.CYLINDER.getValue() ; // | 128;
    public static final int SPHERE_CYLINDER = ShapeTypes.SPHERE.getValue() | ShapeTypes.CYLINDER.getValue() ; //| 129;
    public static final int PLANE_CYLINDER = ShapeTypes.PLANE.getValue()| ShapeTypes.CYLINDER.getValue() ; //| 130;
    public static final int BOX_CYLINDER = ShapeTypes.BOX.getValue() | ShapeTypes.CYLINDER.getValue() ; //| 132;
    public static final int CONVEX_CYLINDER = ShapeTypes.CONVEXPOLYHEDRON.getValue() | ShapeTypes.CYLINDER.getValue() ;// | 144;
    public static final int HEIGHTFIELD_CYLINDER = ShapeTypes.HEIGHTFIELD.getValue() | ShapeTypes.CYLINDER.getValue() ;// | 160;
    public static final int PARTICLE_CYLINDER = ShapeTypes.PARTICLE.getValue() | ShapeTypes.CYLINDER.getValue() ;// | 192;
    public static final int SPHERE_TRIMESH = ShapeTypes.SPHERE.getValue() | ShapeTypes.TRIMESH.getValue();// | 257;
    public static final int PLANE_TRIMESH = ShapeTypes.PLANE.getValue() | ShapeTypes.TRIMESH.getValue();// | 258;

    public static int getSphereSphere() {
        return SPHERE_SPHERE;
    }

    public static int getSpherePlane() {
        return SPHERE_PLANE;
    }

    public static int getBoxBox() {
        return BOX_BOX;
    }

    public static int getSphereBox() {
        return SPHERE_BOX;
    }

    public static int getPlaneBox() {
        return PLANE_BOX;
    }

    public static int getConvexConvex() {
        return CONVEX_CONVEX;
    }

    public static int getSphereConvex() {
        return SPHERE_CONVEX;
    }

    public static int getPlaneConvex() {
        return PLANE_CONVEX;
    }

    public static int getBoxConvex() {
        return BOX_CONVEX;
    }

    public static int getSphereHeightfield() {
        return SPHERE_HEIGHTFIELD;
    }

    public static int getBoxHeightfield() {
        return BOX_HEIGHTFIELD;
    }

    public static int getConvexHeightfield() {
        return CONVEX_HEIGHTFIELD;
    }

    public static int getSphereParticle() {
        return SPHERE_PARTICLE;
    }

    public static int getPlaneParticle() {
        return PLANE_PARTICLE;
    }

    public static int getBoxParticle() {
        return BOX_PARTICLE;
    }

    public static int getConvexParticle() {
        return CONVEX_PARTICLE;
    }

    public static int getCylinderCylinder() {
        return CYLINDER_CYLINDER;
    }

    public static int getSphereCylinder() {
        return SPHERE_CYLINDER;
    }

    public static int getPlaneCylinder() {
        return PLANE_CYLINDER;
    }

    public static int getBoxCylinder() {
        return BOX_CYLINDER;
    }

    public static int getConvexCylinder() {
        return CONVEX_CYLINDER;
    }

    public static int getHeightfieldCylinder() {
        return HEIGHTFIELD_CYLINDER;
    }

    public static int getParticleCylinder() {
        return PARTICLE_CYLINDER;
    }

    public static int getSphereTrimesh() {
        return SPHERE_TRIMESH;
    }

    public static int getPlaneTrimesh() {
        return PLANE_TRIMESH;
    }
}
