import unittest

from fly_in import parse_map, shortest_path, simulate


LINEAR_MAP = """
nb_drones: 2
start_hub: start 0 0 [color=green]
hub: middle 1 0 [color=blue]
end_hub: goal 2 0 [color=red]
connection: start-middle
connection: middle-goal
"""


class FlyInTests(unittest.TestCase):
    def test_parse_and_find_shortest_path(self):
        map_data = parse_map(LINEAR_MAP)

        self.assertEqual(map_data.drones, 2)
        self.assertEqual(shortest_path(map_data), ["start", "middle", "goal"])

    def test_simulate_delivers_all_drones(self):
        turns = simulate(parse_map(LINEAR_MAP))

        self.assertEqual(
            turns,
            [
                "D1-middle",
                "D1-goal D2-middle",
                "D2-goal",
            ],
        )

    def test_blocked_hub_is_unreachable(self):
        map_text = """
nb_drones: 1
start_hub: start 0 0
hub: blocked 1 0 [zone=blocked]
end_hub: goal 2 0
connection: start-blocked
connection: blocked-goal
"""
        map_data = parse_map(map_text)

        self.assertIsNone(shortest_path(map_data))
        with self.assertRaises(ValueError):
            simulate(map_data)


if __name__ == "__main__":
    unittest.main()
