package collision;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import math.Vec3;
import world.World;
import objects.Body;
import shapes.Box;
import shapes.ConvexPolyhedron;
import shapes.Heightfield;
import shapes.Plane;
import shapes.Shape;
import shapes.ShapeTypes;
import shapes.Sphere;
import shapes.Trimesh;
import math.Quaternion;
import math.Transform;

/**
 * RAY_MODES
 */
public class Ray {

	protected Vec3 from;
	protected Vec3 to;
	protected Vec3 direction;
	/**
   * The precision of the ray. Used when checking parallelity etc.
   * @default 0.0001
   */
	protected double precision;
	/**
   * Set to `false` if you don't want the Ray to take `collisionResponse` flags into account on bodies and shapes.
   * @default true
   */
	protected boolean checkCollisionResponse;
	 /**
   * If set to `true`, the ray skips any hits with normal.dot(rayDirection) < 0.
   * @default false
   */
	protected boolean skipBackfaces;
	//@default -1
	protected int collisionFilterMask;
	//default -1
	protected int collisionFilterGroup;

	/**
   * The intersection mode. Should be Ray.ANY, Ray.ALL or Ray.CLOSEST.
   * @default RAY.ANY
   */
	protected RayModes mode;
	/**
   * Current result object.
   */
	protected RaycastResult result;
	/**
   * Will be set to `true` during intersectWorld() if the ray hit anything.
   */
	protected boolean hasHit;
	 /**
   * User-provided result callback. Will be used if mode is Ray.ALL.
   */
	protected RaycastCallback callback;

	static RayModes CLOSEST = RayModes.CLOSEST;

	static RayModes ANY = RayModes.ANY;

	static RayModes ALL = RayModes.ALL;

	// public Object getTypeIntersection(int type) {
    //     switch (type) {
    //         case ShapeTypes.SPHERE.getValue():
    //             return _intersectSphere();
    //         case Shape.types.PLANE:
    //             return _intersectPlane();
    //         case Shape.types.BOX:
    //             return _intersectBox();
    //         case Shape.types.CYLINDER:
    //         case Shape.types.CONVEXPOLYHEDRON:
    //             return _intersectConvex();
    //         case Shape.types.HEIGHTFIELD:
    //             return _intersectHeightfield();
    //         case Shape.types.TRIMESH:
    //             return _intersectTrimesh();
    //         default:
    //             return null;
    //     }
    // }

	// constructor(from = new Vec3(), to = new Vec3())
	public Ray() {
		this(new Vec3(), new Vec3());
	}

	public Ray(Vec3 from, Vec3 to) {
		this.from = from;
		this.to = to;
		this.direction = new Vec3();
		this.precision = 0.0001;
		this.checkCollisionResponse = true;
		this.skipBackfaces = false;
		this.collisionFilterMask = -1;
		this.collisionFilterGroup = -1;
		this.mode = RayModes.ANY;
		this.result = new RaycastResult();
		this.hasHit = false;
		this.callback = DUMMY_CALLBACK;
		this.direction = to.vsub(from).unit();
	}

	/**
   * Do itersection against all bodies in the given World.
   * @return True if the ray hit anything, otherwise false.
   */
	public boolean intersectWorld(World world, RayOptions options) {

		List<Body> bodies = world.bodies;
		//this.callback = callback;
		this.hasHit = false;
		this.result = new RaycastResult();

		for (Body body : bodies) {
			if (checkCollisionResponse && !body.collisionResponse) {
				continue;
			}

			if ((collisionFilterGroup & body.collisionFilterMask) == 0
					|| (body.collisionFilterGroup & collisionFilterMask) == 0) {
				continue;
			}

			intersectBody(body);
		}

		return hasHit;
	}

	public void intersectBody(Body body) {
		intersectBody(body, new RaycastResult());
	}

	/**
	 * Shoot a ray at a body, get back information about the hit.
	 * 
	 * @param body   The Body to intersect with.
	 * @param result Set the result property of the Ray instead.
	 */
	public void intersectBody(Body body, RaycastResult result) {
		if (result != null) {
			this.result = result;
			updateDirection();
		}

		boolean checkCollisionResponse = this.checkCollisionResponse;

		if (checkCollisionResponse && !body.collisionResponse) {
			return;
		}

		if ((this.collisionFilterGroup & body.collisionFilterMask) == 0
				|| (body.collisionFilterGroup & this.collisionFilterMask) == 0) {
			return;
		}

		Vec3 xi = intersectBody_xi;
		Quaternion qi = intersectBody_qi;

		for (int i = 0, N = body.shapes.size(); i < N; i++) {
			Shape shape = body.shapes.get(i);

			if (checkCollisionResponse && !shape.collisionResponse) {
				continue; // Skip
			}

			body.quaternion.mult(body.shapeOrientations.get(i), qi);
			body.quaternion.vmult(body.shapeOffsets.get(i), xi);
			xi.vadd(body.position, xi);

			intersectShape(shape, qi, xi, body);

			if (this.result.shouldStop) {
				break;
			}
		}
	}

	/**
	 * Shoot a ray at an array of bodies, get back information about the hit.
	 * 
	 * @param bodies An array of Body objects.
	 * @param result Set the result property of the Ray instead.
	 */
	public void intersectBodies(List<Body> bodies, RaycastResult result) {
		if (result != null) {
			this.result = result;
			updateDirection();
		}

		for (int i = 0, l = bodies.size(); !this.result.shouldStop && i < l; i++) {
			intersectBody(bodies.get(i), null);
		}
	}

	/**
	 * Updates the direction vector.
	 */
	private void updateDirection() {
		this.to.vsub(this.from, this.direction);
		this.direction.normalize();
	}

	private void intersectShape(Shape shape, Quaternion quat, Vec3 position, Body body) {
		Vec3 from = this.from;

		// Checking boundingSphere
		double distance = distanceFromIntersection(from, this.direction, position);
		if (distance > shape.boundingSphereRadius) {
			return;
		}

		ShapeTypes shapeType = shape.type;

		// if(shape.type.SPHERE != null) (shapeType.equals(Shape.types.SPHERE))
		if (shapeType == ShapeTypes.SPHERE) {
			_intersectSphere((Sphere) shape, quat, position, body, shape);
		} else if (shapeType == ShapeTypes.PLANE) {
			_intersectPlane((Plane) shape, quat, position, body, shape);
		} else if (shapeType.equals(ShapeTypes.BOX)) {
			_intersectBox((Box) shape, quat, position, body, shape);
		} else if (shapeType.equals(ShapeTypes.CYLINDER) || shapeType.equals(ShapeTypes.CONVEXPOLYHEDRON)) {
			_intersectConvex((ConvexPolyhedron) shape, quat, position, body, shape, null);
		} else if (shapeType.equals(ShapeTypes.HEIGHTFIELD)) {
			_intersectHeightfield((Heightfield) shape, quat, position, body, shape);
		} else if (shapeType.equals(ShapeTypes.TRIMESH)) {
			_intersectTrimesh((Trimesh) shape, quat, position, body, shape, null);
		}
	}

	private void _intersectBox(Box box, Quaternion quat, Vec3 position, Body body, Shape reportedShape) {
		_intersectConvex(box.convexPolyhedronRepresentation, quat, position, body, reportedShape);
	}

	private void _intersectPlane(Plane shape, Quaternion quat, Vec3 position, Body body, Shape reportedShape) {
		Vec3 from = this.from;
		Vec3 to = this.to;
		Vec3 direction = this.direction;

		// Get plane normal
		Vec3 worldNormal = new Vec3(0, 0, 1);
		quat.vmult(worldNormal, worldNormal);

		Vec3 len = new Vec3();
		from.vsub(position, len);
		double planeToFrom = len.dot(worldNormal);
		to.vsub(position, len);
		double planeToTo = len.dot(worldNormal);

		if (planeToFrom * planeToTo > 0) {
			// "from" and "to" are on the same side of the plane... bail out
			return;
		}

		if (from.distanceTo(to) < planeToFrom) {
			return;
		}

		double n_dot_dir = worldNormal.dot(direction);

		if (Math.abs(n_dot_dir) < this.precision) {
			// No intersection
			return;
		}

		Vec3 planePointToFrom = new Vec3();
		Vec3 dir_scaled_with_t = new Vec3();
		Vec3 hitPointWorld = new Vec3();

		from.vsub(position, planePointToFrom);
		double t = -worldNormal.dot(planePointToFrom) / n_dot_dir;
		direction.scale(t, dir_scaled_with_t);
		from.vadd(dir_scaled_with_t, hitPointWorld);

		reportIntersection(worldNormal, hitPointWorld, reportedShape, body, -1);
	}

	/**
	 * Get the world AABB of the ray.
	 */
	public void getAABB(AABB aabb) {
		Vec3 lowerBound = aabb.lowerBound;
		Vec3 upperBound = aabb.upperBound;
		Vec3 to = this.to;
		Vec3 from = this.from;
		lowerBound.x = Math.min(to.x, from.x);
		lowerBound.y = Math.min(to.y, from.y);
		lowerBound.z = Math.min(to.z, from.z);
		upperBound.x = Math.max(to.x, from.x);
		upperBound.y = Math.max(to.y, from.y);
		upperBound.z = Math.max(to.z, from.z);
	}

	private void _intersectHeightfield(Heightfield shape, Quaternion quat, Vec3 position, Body body,Shape reportedShape) {
		double[][] data = shape.data;
		double w = shape.elementSize;
		// Convert the ray to local heightfield coordinates
		Ray localRay = intersectHeightfield_localRay;
		localRay.from.copy(this.from);
		localRay.to.copy(this.to);
		Transform.pointToLocalFrame(position, quat, localRay.from, localRay.from);
		Transform.pointToLocalFrame(position, quat, localRay.to, localRay.to);
		localRay.updateDirection();

		// Get the index of the data points to test against
		int[] index = intersectHeightfield_index; //new int[2];
		int iMinX;
		int iMinY;
		int iMaxX;
		int iMaxY;

		// Set to max
		iMinX = iMinY = 0;
		iMaxX = iMaxY = data.length - 1;

		AABB aabb = new AABB();
		localRay.getAABB(aabb);

		shape.getIndexOfPosition(aabb.lowerBound.x, aabb.lowerBound.y, index, true);
		iMinX = Math.max(iMinX, index[0]);
		iMinY = Math.max(iMinY, index[1]);
		shape.getIndexOfPosition(aabb.upperBound.x, aabb.upperBound.y, index, true);
		iMaxX = Math.min(iMaxX, index[0] + 1);
		iMaxY = Math.min(iMaxY, index[1] + 1);

		for (int i = iMinX; i < iMaxX; i++) {
			for (int j = iMinY; j < iMaxY; j++) {
				if (this.result.shouldStop) {
					return;
				}

				shape.getAabbAtIndex(i, j, aabb);
				if (!aabb.overlapsRay(localRay)) {
					continue;
				}

				// Lower triangle
				shape.getConvexTrianglePillar(i, j, false);
				Vec3 worldPillarOffset = new Vec3();
				Transform.pointToWorldFrame(position, quat, shape.pillarOffset, worldPillarOffset);
				_intersectConvex(shape.pillarConvex, quat, worldPillarOffset, body, reportedShape,
						intersectConvexOptions);

				if (this.result.shouldStop) {
					return;
				}

				// Upper triangle
				shape.getConvexTrianglePillar(i, j, true);
				Transform.pointToWorldFrame(position, quat, shape.pillarOffset, worldPillarOffset);
				_intersectConvex(shape.pillarConvex, quat, worldPillarOffset, body, reportedShape,
						intersectConvexOptions);
			}
		}
	}

	private void _intersectSphere(Sphere sphere, Quaternion quat, Vec3 position, Body body, Shape reportedShape) {
		Vec3 from = this.from;
		Vec3 to = this.to;
		double r = sphere.radius;

		double a = Math.pow((to.x - from.x), 2) + Math.pow((to.y - from.y), 2) + Math.pow((to.z - from.z), 2);
		double b = 2 * ((to.x - from.x) * (from.x - position.x) + (to.y - from.y) * (from.y - position.y)
				+ (to.z - from.z) * (from.z - position.z));
		double c = Math.pow((from.x - position.x), 2) + Math.pow((from.y - position.y), 2)
				+ Math.pow((from.z - position.z), 2) - Math.pow(r, 2);

		double delta = Math.pow(b, 2) - 4 * a * c;

		Vec3 intersectionPoint = Ray_intersectSphere_intersectionPoint;
		Vec3 normal = Ray_intersectSphere_normal;

		if (delta < 0) {
			// No intersection
			return;
		} else if (delta == 0) {
			// single intersection point
			from.lerp(to, delta, intersectionPoint);

			intersectionPoint.vsub(position, normal);
			normal.normalize();

			reportIntersection(normal, intersectionPoint, reportedShape, body, -1);
		} else {
			double d1 = (-b - Math.sqrt(delta)) / (2 * a);
			double d2 = (-b + Math.sqrt(delta)) / (2 * a);

			if (d1 >= 0 && d1 <= 1) {
				from.lerp(to, d1, intersectionPoint);
				intersectionPoint.vsub(position, normal);
				normal.normalize();
				reportIntersection(normal, intersectionPoint, reportedShape, body, -1);
			}

			if (this.result.shouldStop) {
				return;
			}

			if (d2 >= 0 && d2 <= 1) {
				from.lerp(to, d2, intersectionPoint);
				intersectionPoint.vsub(position, normal);
				normal.normalize();
				reportIntersection(normal, intersectionPoint, reportedShape, body, -1);
			}
		}
	}
	
	
	private void _intersectConvex(ConvexPolyhedron shape, Quaternion quat, Vec3 position, Body body,
			Shape reportedShape) {
		_intersectConvex(shape, quat,position , body, reportedShape,null);
	}
	
	
	private void _intersectConvex(ConvexPolyhedron shape, Quaternion quat, Vec3 position, Body body,
			Shape reportedShape, Map<String, Object> options) {
		Vec3 minDistNormal = intersectConvex_minDistNormal;
		Vec3 normal = intersectConvex_normal;
		Vec3 vector = intersectConvex_vector;
		Vec3 minDistIntersect = intersectConvex_minDistIntersect;
		List<Integer> faceList = (options != null && options.containsKey("faceList"))
				? (List<Integer>) options.get("faceList")
				: null;

		// Checking faces
		List<int[]> faces = shape.faces;

		List<Vec3> vertices = shape.vertices;
		List<Vec3> normals = shape.faceNormals;
		Vec3 direction = this.direction;

		Vec3 from = this.from;
		Vec3 to = this.to;
		double fromToDistance = from.distanceTo(to);

		double minDist = -1;
		int Nfaces = (faceList != null) ? faceList.size() : faces.size();
		RaycastResult result = this.result;

		for (int j = 0; !result.shouldStop && j < Nfaces; j++) {
			int fi = (faceList != null) ? faceList.get(j) : j;

			int[] face = faces.get(fi);
			Vec3 faceNormal = normals.get(fi);
			Quaternion q = quat;
			Vec3 x = position;

			// Determine if ray intersects the plane of the face.
			// Note: this works regardless of the direction of the face normal.

			// Get the plane point in world coordinates...
			vector.copy(vertices.get(face[0]));
			q.vmult(vector, vector);
			vector.vadd(x, vector);

			// ...but make it relative to the ray from. We'll fix this later.
			vector.vsub(from, vector);

			// Get the plane normal.
			q.vmult(faceNormal, normal);

			// If this dot product is negative, we have something interesting.
			double dot = direction.dot(normal);

			// Bail out if ray and plane are parallel.
			if (Math.abs(dot) < this.precision) {
				continue;
			}

			// Calculate the distance to the plane.
			double scalar = normal.dot(vector) / dot;

			// If negative distance, then the plane is behind the ray.
			if (scalar < 0) {
				continue;
			}

			// Intersection point is from + direction * scalar.
			direction.scale(scalar, intersectPoint);
			intersectPoint.vadd(from, intersectPoint);

			// "a" is the point we compare points "b" and "c" with.
			Vec3 a = vertices.get(face[0]);
			q.vmult(a, a);
			x.vadd(a, a);

			for (int i = 1; !result.shouldStop && i < face.length - 1; i++) {
				// Transform 3 vertices to world coordinates.
				b.copy(vertices.get(face[i]));
				c.copy(vertices.get(face[i + 1]));
				q.vmult(b, b);
				q.vmult(c, c);
				x.vadd(b, b);
				x.vadd(c, c);

				double distance = intersectPoint.distanceTo(from);

				if (!(Ray.pointInTriangle(intersectPoint, a, b, c) || Ray.pointInTriangle(intersectPoint, b, a, c))
						|| distance > fromToDistance) {
					continue;
				}

				this.reportIntersection(normal, intersectPoint, reportedShape, body, fi);
			}
		}
	}

	
  /**
   * @todo Optimize by transforming the world to local space first.
   * @todo Use Octree lookup
   */
	private void _intersectTrimesh(Trimesh mesh, Quaternion quat, Vec3 position, Body body, Shape reportedShape,Map<String, Object> options) {
		Vec3 normal = intersectTrimesh_normal;
		List<Integer> triangles = intersectTrimesh_triangles;
		Transform treeTransform = intersectTrimesh_treeTransform;
		Vec3 vector = intersectConvex_vector;
		Vec3 localDirection = intersectTrimesh_localDirection;
		Vec3 localFrom = intersectTrimesh_localFrom;
		Vec3 localTo = intersectTrimesh_localTo;
		Vec3 worldIntersectPoint = intersectTrimesh_worldIntersectPoint;
		Vec3 worldNormal = intersectTrimesh_worldNormal;

		// Checking faces
		int[] indices = mesh.indices; // int to short
		double[] vertices = mesh.vertices; // list<Vec3> to float
		// List<Vec3> normals = mesh.getFaceNormals();
		Vec3 from = this.from;
		Vec3 to = this.to;
		Vec3 direction = this.direction;

		treeTransform.position.copy(position);
		treeTransform.quaternion.copy(quat);

		// Transform the ray to local space.
		Transform.vectorToLocalFrame(position, quat, direction, localDirection);
		Transform.pointToLocalFrame(position, quat, from, localFrom);
		Transform.pointToLocalFrame(position, quat, to, localTo);

		localTo.x *= mesh.scale.x;
		localTo.y *= mesh.scale.y;
		localTo.z *= mesh.scale.z;
		localFrom.x *= mesh.scale.x;
		localFrom.y *= mesh.scale.y;
		localFrom.z *= mesh.scale.z;

		localTo.vsub(localFrom, localDirection);
		localDirection.normalize();

		double fromToDistanceSquared = localFrom.distanceSquared(localTo);

		mesh.tree.rayQuery(this, treeTransform, triangles);

		for (int i = 0, N = triangles.size(); !this.result.shouldStop && i != N; i++) {
			int trianglesIndex = triangles.get(i);

			mesh.getNormal(trianglesIndex, normal);

			// Determine if the ray intersects the plane of the face.
			// Note: this works regardless of the direction of the face normal.

			// Get the plane point in world coordinates...
			// Vec3 a = vertices.get(indices[trianglesIndex * 3]);
			Vec3 a = new Vec3();
			mesh.getVertex(indices[trianglesIndex * 3], a);
			// ...but make it relative to the ray "from." We'll fix this later.
			a.vsub(localFrom, vector);

			// If this dot product is negative, we have something interesting.
			double dot = localDirection.dot(normal);

			// Bail out if the ray and plane are parallel.
			if (Math.abs(dot) < this.precision) {
				continue;
			}

			// Calculate the distance to the plane.
			double scalar = normal.dot(vector) / dot;

			// If negative distance, then the plane is behind the ray.
			if (scalar < 0) {
				continue;
			}

			// Intersection point is from + direction * scalar.
			localDirection.scale(scalar, intersectPoint);
			intersectPoint.vadd(localFrom, intersectPoint);

			// Get triangle vertices

			Vec3 b = new Vec3();
			Vec3 c = new Vec3();
			mesh.getVertex(indices[trianglesIndex * 3 + 1], b);
			mesh.getVertex(indices[trianglesIndex * 3 + 2], c);

			double squaredDistance = intersectPoint.distanceSquared(localFrom);

			if (!(Ray.pointInTriangle(intersectPoint, b, a, c) || Ray.pointInTriangle(intersectPoint, a, b, c))
					|| squaredDistance > fromToDistanceSquared) {
				continue;
			}

			// Transform intersect point and normal to world coordinates
			Transform.vectorToWorldFrame(quat, normal, worldNormal);
			Transform.pointToWorldFrame(position, quat, intersectPoint, worldIntersectPoint);
			this.reportIntersection(worldNormal, worldIntersectPoint, reportedShape, body, trianglesIndex);
		}
		triangles.clear();
	}

	private  Vec3 intersectPoint = new Vec3();
	protected  static Vec3 v0 = new Vec3();
	protected  static Vec3 v1 = new Vec3();
	protected  static Vec3 v2 = new Vec3();

	private  Vec3 intersectConvex_minDistNormal = new Vec3();
	private  Vec3 intersectConvex_normal = new Vec3();
	private  Vec3 intersectConvex_vector = new Vec3();
	private  Vec3 intersectConvex_minDistIntersect = new Vec3();

	private  Vec3 intersectTrimesh_normal = new Vec3();
	private  List<Integer> intersectTrimesh_triangles = new ArrayList<>();
	private  Transform intersectTrimesh_treeTransform = new Transform();
	private  Vec3 intersectTrimesh_localDirection = new Vec3();
	private  Vec3 intersectTrimesh_localFrom = new Vec3();
	private  Vec3 intersectTrimesh_localTo = new Vec3();
	private  Vec3 intersectTrimesh_worldIntersectPoint = new Vec3();
	private  Vec3 intersectTrimesh_worldNormal = new Vec3();
	//private final AABB intersectTrimesh_localAABB = new AABB();

	private  Vec3 intersectBody_xi = new Vec3();
	private  Quaternion intersectBody_qi = new Quaternion();

	private  Vec3 a = new Vec3();
	private  Vec3 b = new Vec3();
	private  Vec3 c = new Vec3();
	//private final Vec3 d = new Vec3();

	//private final RaycastResult tmpRaycastResult = new RaycastResult();

	private  Map<String, Object> intersectConvexOptions = new HashMap<>();
	private  Vec3 worldPillarOffset = new Vec3();
	private  Ray intersectHeightfield_localRay = new Ray();
	private  int[] intersectHeightfield_index = new int[2];
	//private final List<Integer> intersectHeightfield_minMax = new ArrayList<>();

	private  Vec3 Ray_intersectSphere_intersectionPoint = new Vec3();
	private  Vec3 Ray_intersectSphere_normal = new Vec3();

	private  AABB tmpAABB = new AABB();
	private  List<Body> tmpArray = new ArrayList<>();

	/**
	 * @return True if the intersections should continue
	 */
	private void reportIntersection(Vec3 normal, Vec3 hitPointWorld, Shape shape, Body body, int hitFaceIndex) {
		Vec3 from = this.from;
		Vec3 to = this.to;
		double distance = from.distanceTo(hitPointWorld);

		// Skip back faces?
		if (this.skipBackfaces && normal.dot(this.direction) > 0) {
			return;
		}

		result.hitFaceIndex = (hitFaceIndex != -1) ? hitFaceIndex : -1;

		switch (this.mode) {
		case ALL:
			hasHit = true;
			result.set(from, to, normal, hitPointWorld, shape, body, distance);
			result.hasHit = true;
			callback.onRaycast(result);
			break;

		case CLOSEST:
			// Store if closer than current closest
			if (distance < result.distance || !result.hasHit) {
				hasHit = true;
				result.hasHit = true;
				result.set(from, to, normal, hitPointWorld, shape, body, distance);
			}
			break;

		case ANY:
			// Report and stop.
			hasHit = true;
			result.hasHit = true;
			result.set(from, to, normal, hitPointWorld, shape, body, distance);
			result.shouldStop = true;
			break;
		}
	}

	  /**
   * As per "Barycentric Technique" as named
   * {@link https://www.blackpawn.com/texts/pointinpoly/default.html here} but without the division
   */
	public static boolean pointInTriangle(Vec3 p, Vec3 a, Vec3 b, Vec3 c) {
		c.vsub(a, v0);
		b.vsub(a, v1);
		p.vsub(a, v2);
		double dot00 = v0.dot(v0);
		double dot01 = v0.dot(v1);
		double dot02 = v0.dot(v2);
		double dot11 = v1.dot(v1);
		double dot12 = v1.dot(v2);
		double u;
		double v;
		return (u = dot11 * dot02 - dot01 * dot12) >= 0 && (v = dot00 * dot12 - dot01 * dot02) >= 0
				&& u + v < dot00 * dot11 - dot01 * dot01;
	}

	private double distanceFromIntersection(Vec3 from, Vec3 direction, Vec3 position) {
		position.vsub(from, v0);
		double dot = v0.dot(direction);

		direction.scale(dot, intersectPoint);
		intersectPoint.vadd(from, intersectPoint);

		double distance = position.distanceTo(intersectPoint);

		return distance;
	}

	// Implement intersectFixture and other necessary methods
	
}
