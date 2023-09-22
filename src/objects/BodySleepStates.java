package objects;
/**
 * BodySleepState enum
 */
public enum BodySleepStates {
    AWAKE(0),
    SLEEPY(1),
    SLEEPING(2);

    private final int value;

    private BodySleepStates(int value) {
        this.value = value;
    }

    public int getValue() {
        return value;
    }
}
