"""ロボマスターモーターの角度管理モジュール."""


class RobomasterAngleTracker:
    """ロボマスターモーターの角度を追跡するクラス."""

    def __init__(self):
        """変数を初期化する."""
        self.last_raw = None
        self.total_angle = 0  # 累積角度

    def update(self, raw_angle):
        """生角度を更新して累積角度を計算する."""
        if self.last_raw is None:
            self.last_raw = raw_angle
            return self.total_angle

        delta = (raw_angle - self.last_raw + 8192) % 8192
        if delta > 4096:
            delta -= 8192  # 巻き戻しを補正

        self.total_angle += delta
        self.last_raw = raw_angle
        return self.total_angle

    def get_angle(self):
        """累積角度を取得する."""
        return self.total_angle

    def get_raw_angle(self):
        """最新の生角度を取得する."""
        return self.last_raw
