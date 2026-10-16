import unittest

from scripts.check_gazebo_macos import _array_values


class GazeboSmokeCheckTests(unittest.TestCase):
    def test_parses_ros_cli_yaml_sequence(self) -> None:
        message = """layout: {}\ndata:\n- 3401.0\n- -1.0\n"""
        self.assertEqual(_array_values(message), [3401.0, -1.0])

    def test_parses_inline_sequence(self) -> None:
        self.assertEqual(_array_values("data: [1.5, 2.5]\n"), [1.5, 2.5])

    def test_rejects_missing_or_invalid_data(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "did not contain"):
            _array_values("layout: {}\n")
        with self.assertRaisesRegex(RuntimeError, "invalid numeric"):
            _array_values("data:\n- not-a-number\n")


if __name__ == "__main__":
    unittest.main()
