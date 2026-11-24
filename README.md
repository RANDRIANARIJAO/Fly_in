# Fly_in

Fly_in is a turn-based drone routing simulation. A fleet of drones travels from
one start hub to one end hub over a network of hubs and connections while
respecting hub capacities and connection throughput.

This implementation uses only the Python standard library and Dijkstra's
algorithm for pathfinding.

## Map format

```text
nb_drones: 12

start_hub: hub 0 0 [color=green zone=normal]
end_hub: goal 10 10 [color=yellow zone=normal]
hub: waypoint 3 4 [zone=normal color=blue max_drones=2]
hub: restricted 5 5 [zone=restricted color=orange]
hub: blocked 7 7 [zone=blocked color=gray]

connection: hub-waypoint
connection: waypoint-goal [max_link_capacity=2]
```

Comments start with `#`. Hub zones may be `normal`, `restricted`, `priority`,
or `blocked`. Blocked hubs cannot be used. Connections are bidirectional.

## Run

```bash
python3 main.py map.txt
```

The program prints one line of drone movements per turn.

## Test

```bash
python3 -m unittest discover -s tests -v
```