"""Core parser and turn-based simulator for Fly_in maps."""

from __future__ import annotations

import heapq
import re
from dataclasses import dataclass, field
@dataclass
class Hub:
    name: str
    x: int
    y: int
    zone: str = "normal"
    max_drones: int | None = None
    color: str = "white"


@dataclass
class Connection:
    left: str
    right: str
    max_link_capacity: int = 1


@dataclass
class Map:
    drones: int
    start: str
    end: str
    hubs: dict[str, Hub] = field(default_factory=dict)
    connections: list[Connection] = field(default_factory=list)

    @property
    def neighbours(self) -> dict[str, list[str]]:
        result = {name: [] for name in self.hubs}
        for connection in self.connections:
            if connection.left not in result or connection.right not in result:
                continue
            if connection.right not in result[connection.left]:
                result[connection.left].append(connection.right)
            if connection.left not in result[connection.right]:
                result[connection.right].append(connection.left)
        return result


_ATTRIBUTE_RE = re.compile(r"(\w+)=(?:\"([^\"]*)\"|'([^']*)'|([^]\s]+))")


def _attributes(text: str) -> dict[str, str]:
    values: dict[str, str] = {}
    for match in _ATTRIBUTE_RE.finditer(text):
        values[match.group(1)] = next(
            value for value in match.groups()[1:] if value is not None
        )
    return values


def parse_map(text: str) -> Map:
    """Parse a Fly_in map and raise ValueError for malformed input."""
    drones: int | None = None
    start: str | None = None
    end: str | None = None
    hubs: dict[str, Hub] = {}
    connections: list[Connection] = []

    for raw_line in text.splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line:
            continue
        if line.startswith("nb_drones:"):
            if drones is not None:
                raise ValueError("nb_drones may only be declared once")
            value = line.partition(":")[2].strip()
            if not value.isdigit() or int(value) < 0:
                raise ValueError("nb_drones must be a non-negative integer")
            drones = int(value)
            continue

        if ":" not in line:
            raise ValueError("invalid map line: " + line)
        kind, body = (part.strip() for part in line.split(":", 1))
        if kind in ("start_hub", "end_hub", "hub"):
            attribute_text = ""
            if "[" in body:
                body, attribute_text = body.split("[", 1)
                attribute_text = attribute_text.rstrip("]").strip()
            fields = body.split()
            if len(fields) < 3:
                raise ValueError("hub declarations require a name and coordinates")
            name = fields[0]
            try:
                x, y = int(fields[1]), int(fields[2])
            except ValueError as exc:
                raise ValueError("hub coordinates must be integers") from exc
            if name in hubs:
                raise ValueError("duplicate hub: " + name)
            attributes = _attributes(attribute_text)
            zone = attributes.get("zone", "normal").lower()
            if zone not in ("normal", "restricted", "priority", "blocked"):
                raise ValueError("invalid zone type: " + zone)
            max_drones: int | None = None
            if "max_drones" in attributes:
                if not attributes["max_drones"].isdigit():
                    raise ValueError("max_drones must be a non-negative integer")
                max_drones = int(attributes["max_drones"])
            hubs[name] = Hub(
                name=name,
                x=x,
                y=y,
                zone=zone,
                max_drones=max_drones,
                color=attributes.get("color", "white"),
            )
            if kind == "start_hub":
                if start is not None:
                    raise ValueError("only one start_hub is allowed")
                start = name
            elif kind == "end_hub":
                if end is not None:
                    raise ValueError("only one end_hub is allowed")
                end = name
            continue

        if kind == "connection":
            attribute_text = ""
            if "[" in body:
                body, attribute_text = body.split("[", 1)
                attribute_text = attribute_text.rstrip("]").strip()
            names = body.strip()
            attributes = _attributes(attribute_text)
            capacity = int(attributes.get("max_link_capacity", "1"))
            if capacity < 1:
                raise ValueError("max_link_capacity must be positive")
            connections.append(Connection("", "", capacity))
            connections[-1].left, connections[-1].right = _split_connection(
                names, hubs
            )
            continue

        raise ValueError("unknown declaration: " + kind)

    if drones is None:
        raise ValueError("nb_drones is required")
    if start is None or end is None:
        raise ValueError("start_hub and end_hub are required")
    if start not in hubs or end not in hubs:
        raise ValueError("start_hub and end_hub must be declared")
    for connection in connections:
        if connection.left not in hubs or connection.right not in hubs:
            raise ValueError("connection references an undeclared hub")
    if hubs[start].zone == "blocked" or hubs[end].zone == "blocked":
        raise ValueError("start_hub and end_hub cannot be blocked")
    return Map(drones, start, end, hubs, connections)


def _split_connection(names: str, hubs: dict[str, Hub]) -> tuple[str, str]:
    if "-" in names:
        for index, character in enumerate(names):
            if character == "-":
                left, right = names[:index], names[index + 1:]
                if left in hubs and right in hubs:
                    return left, right
    parts = names.split()
    if len(parts) == 2:
        return parts[0], parts[1]
    raise ValueError("invalid connection: " + names)


def shortest_path(map_data: Map) -> list[str] | None:
    """Return a lowest-cost path, or None when no route exists."""
    distances = {name: float("inf") for name in map_data.hubs}
    previous: dict[str, str] = {}
    distances[map_data.start] = 0
    queue: list[tuple[float, str]] = [(0, map_data.start)]

    while queue:
        distance, name = heapq.heappop(queue)
        if distance != distances[name]:
            continue
        if name == map_data.end:
            break
        if map_data.hubs[name].zone == "blocked":
            continue
        for neighbour in map_data.neighbours[name]:
            if map_data.hubs[neighbour].zone == "blocked":
                continue
            step_cost = 2 if map_data.hubs[neighbour].zone == "restricted" else 1
            new_distance = distance + step_cost
            if new_distance < distances[neighbour]:
                distances[neighbour] = new_distance
                previous[neighbour] = name
                heapq.heappush(queue, (new_distance, neighbour))

    if map_data.end not in previous and map_data.start != map_data.end:
        return None
    path = [map_data.end]
    while path[-1] != map_data.start:
        path.append(previous[path[-1]])
    path.reverse()
    return path


def simulate(map_data: Map) -> list[str]:
    """Simulate one drone step per turn and return movement log lines."""
    path = shortest_path(map_data)
    if map_data.drones and path is None:
        raise ValueError("no route from start_hub to end_hub")

    positions = [map_data.start] * map_data.drones
    progress = [0] * map_data.drones
    waiting = [0] * map_data.drones
    delivered = 0
    turns: list[str] = []

    while delivered < map_data.drones:
        if len(turns) > 10000:
            raise RuntimeError("simulation exceeded 10000 turns")
        occupancy = {name: 0 for name in map_data.hubs}
        for position in positions:
            if position != map_data.end:
                occupancy[position] = occupancy.get(position, 0) + 1

        link_usage: dict[tuple[str, str], int] = {}
        movements: list[str] = []
        capacities = {name: hub.max_drones for name, hub in map_data.hubs.items()}
        moved = False

        for drone_id in range(map_data.drones):
            if progress[drone_id] >= len(path) - 1:
                continue
            if waiting[drone_id] > 0:
                waiting[drone_id] -= 1
                continue

            source = positions[drone_id]
            target = path[progress[drone_id] + 1]
            if source not in path or target not in path:
                continue
            key = tuple(sorted((source, target)))
            capacity = next(
                (
                    connection.max_link_capacity
                    for connection in map_data.connections
                    if {connection.left, connection.right} == set(key)
                ),
                1,
            )
            if link_usage.get(key, 0) >= capacity:
                continue
            target_capacity = capacities[target]
            if target != map_data.end and target_capacity is not None:
                if occupancy[target] >= target_capacity:
                    continue
            link_usage[key] = link_usage.get(key, 0) + 1
            occupancy[source] -= 1
            occupancy[target] = occupancy.get(target, 0) + 1
            positions[drone_id] = target
            progress[drone_id] += 1
            if map_data.hubs[target].zone == "restricted":
                waiting[drone_id] = 1
            movements.append(f"D{drone_id + 1}-{target}")
            moved = True
            if target == map_data.end:
                delivered += 1

        if movements:
            turns.append(" ".join(movement for movement in movements))
        if delivered == map_data.drones:
            break
        if not moved:
            raise RuntimeError("drones are blocked; no movement is possible")

    return turns
