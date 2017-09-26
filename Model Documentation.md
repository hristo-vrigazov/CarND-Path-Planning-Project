# Model documentation

## General points

### Setting up

First, we create an instance of the `World` by the given map filename. A `PathPlanner` object
is then created using the instance of the `World`.

### Entry point
The entry point is `PathPlanner#plan`, which accepts as input `TelemetryData`, which
is a serialized class using the converters defined n `JsonConverters`. The `PathPlanner#plan`
returns the path that the car should follow and those points are sent to the simulator.

