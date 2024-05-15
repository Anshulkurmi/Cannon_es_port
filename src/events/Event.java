package events;

public class Event {
	 	protected String type;
	     EventTarget target;

	    public Event(String type, EventTarget target) {
	        this.type = type;
	        this.target = target;
	    }

	    public String getType() {
	        return type;
	    }

	    public EventTarget getTarget() {
	        return target;
	    }
}
