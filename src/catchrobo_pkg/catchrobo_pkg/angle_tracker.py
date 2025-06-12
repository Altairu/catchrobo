# -*- coding: utf-8 -*-
"""
RobomasterAngleTracker クラスは、ロボマスモーターの角度を管理するためのクラスです。
このクラスは、無限角度の計測を可能にします。
"""


class RobomasterAngleTracker:
    def __init__(self):
        # 最後に受信した生角度
        self.last_raw = None
        # 累積角度（0基準）
        self.total_angle = 0

    def update(self, raw_angle):
        """
        モーターから受信した生角度（0〜8191）を更新します。

        :param raw_angle: モーターから受信した生角度
        :return: 累積角度
        """
        if self.last_raw is None:
            self.last_raw = raw_angle
            return self.total_angle

        # 差分を計算（巻き戻し対応）
        delta = (raw_angle - self.last_raw + 8192) % 8192
        if delta > 4096:
            delta -= 8192

        # 累積角度を更新
        self.total_angle += delta
        self.last_raw = raw_angle
        return self.total_angle

    def get_angle(self):
        """
        累積角度を取得します。

        :return: 累積角度（8192 = 360度）
        """
        return self.total_angle

    def get_raw_angle(self):
        """
        最新の生角度を取得します。

        :return: 生角度（0〜8191）
        """
        return self.last_raw
