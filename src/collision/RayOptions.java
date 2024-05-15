package collision;

import math.Vec3;


public class RayOptions {
    Vec3 from;
     Vec3 to;
    RayModes mode;
     RaycastResult result;
    /**
   * If set to `true`, the ray skips any hits with normal.dot(rayDirection) < 0.
   * @default false
   */
    boolean skipBackfaces;
    /**
   * collisionFilterMask
   * @default -1
   */
     int collisionFilterMask;
    /**
   * collisionFilterGroup
   * @default -1
   */
     int collisionFilterGroup;
    /**
   * Set to `false` if you don't want the Ray to take `collisionResponse` flags into account on bodies and shapes.
   * @default true
   */
     boolean checkCollisionResponse;

    /**
   * callback
   */
     RaycastCallback callback;

    public RayOptions() {
        // Default values
        this.from = null;
        this.to = null;
        this.mode = RayModes.ANY;
        this.result = null;
        this.skipBackfaces = false;
        this.collisionFilterMask = -1;
        this.collisionFilterGroup = -1;
        this.checkCollisionResponse = true;
        this.callback = null;
    }

    public RayOptions(Vec3 from , Vec3 to , RayModes mode , RaycastResult result ,boolean skipBackfaces,int collisionFilterMask,int collisionFilterGroup,boolean checkCollisionResponse,RaycastCallback callback){
        this.from = from ;
        this.to = to ;
        this.mode = mode ;
        this.result = result ;
        this.skipBackfaces = skipBackfaces;
        this.collisionFilterMask = collisionFilterMask;
        this.collisionFilterGroup= collisionFilterGroup;
        this.checkCollisionResponse = checkCollisionResponse ;
        this.callback = callback;
    }

    public Vec3 getFrom() {
        return from;
    }

    public void setFrom(Vec3 from) {
        this.from = from;
    }

    public Vec3 getTo() {
        return to;
    }

    public void setTo(Vec3 to) {
        this.to = to;
    }

    public RayModes getMode() {
        return mode;
    }

    public void setMode(RayModes mode) {
        this.mode = mode;
    }

    public RaycastResult getResult() {
        return result;
    }

    public void setResult(RaycastResult result) {
        this.result = result;
    }

    public boolean isSkipBackfaces() {
        return skipBackfaces;
    }

    public void setSkipBackfaces(boolean skipBackfaces) {
        this.skipBackfaces = skipBackfaces;
    }

    public int getCollisionFilterMask() {
        return collisionFilterMask;
    }

    public void setCollisionFilterMask(int collisionFilterMask) {
        this.collisionFilterMask = collisionFilterMask;
    }

    public int getCollisionFilterGroup() {
        return collisionFilterGroup;
    }

    public void setCollisionFilterGroup(int collisionFilterGroup) {
        this.collisionFilterGroup = collisionFilterGroup;
    }

    public boolean isCheckCollisionResponse() {
        return checkCollisionResponse;
    }

    public void setCheckCollisionResponse(boolean checkCollisionResponse) {
        this.checkCollisionResponse = checkCollisionResponse;
    }

    public RaycastCallback getCallback() {
        return callback;
    }

    public void setCallback(RaycastCallback callback) {
        this.callback = callback;
    }
}

