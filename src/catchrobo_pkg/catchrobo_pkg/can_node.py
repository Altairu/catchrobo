"""CANノードのサンプル実装."""

from .angle_tracker import RobomasterAngleTracker


def main():
    """エントリポイント."""
    tracker = RobomasterAngleTracker()
    print(tracker.get_angle())


if __name__ == "__main__":
    main()
