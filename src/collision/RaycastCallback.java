package collision;

@FunctionalInterface
public interface RaycastCallback {
    void onRaycast(RaycastResult result);
}
