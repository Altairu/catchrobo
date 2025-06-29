# キャチロボ用ROS2パッケージ

## 概要
このパッケージは、キャチロボの制御を行うためのROS2ノード群を提供します。各ノードは特定の機能を担当し、ロボットの動作を効率的に管理します。

## ノード一覧

### 1. can_node
- **説明**: マイコンやロボマスモーターとCAN通信を行います。
- **主な機能**:
  - データの送受信
  - デバッグ情報の処理
  - リミットスイッチの状態をトピックで送信
- **使用例**:
  ```python
  can_node = CANNode()
  can_node.send_data(0x200, [0x01, 0x02, 0x03])
  ```

### 2. pid_node
- **説明**: ロボマスモーターのPID制御を行います。
- **主な機能**:
  - 比例ゲインと不感帯の設定
  - トルク値の計算
  - PID現在値のトピック受信
- **使用例**:
  ```python
  pid_node = PIDNode()
  pid_node.set_target(100)
  ```

### 3. planning_node
- **説明**: ロボットの自動制御を行います。
- **主な機能**:
  - 動作番号と特別動作番号に基づく制御
  - リミットスイッチの状態を利用した制御
- **使用例**:
  ```python
  planning_node = PlanningNode()
  planning_node.action_number = 2
  planning_node.execute_action()
  ```

### 4. web_socket_node
- **説明**: WebSocket通信を行い、特別動作番号を操作します。
- **主な機能**:
  - スマートフォンとPC間の通信
- **使用例**:
  ```python
  node = WebSocketNode(host="0.0.0.0", port=8080)
  node.start_server()
  ```

### 5. dualshock3_node
- **説明**: DualShock3コントローラを使用して動作番号を操作します。
- **主な機能**:
  - ボタン操作による動作番号の増減
- **使用例**:
  ```python
  node = DualShock3Node()
  rclpy.spin(node)
  ```

## アクチュエータ・センサー一覧
- **ロボマスモーター**: 1～5
- **モーター**: 1～3
- **サーボモーター**: 1～12
- **リミットスイッチ**: 1～4

## インストール方法
1. 必要な依存関係をインストールします。
2. このパッケージをROS2ワークスペースにクローンします。
3. `colcon build`を実行してビルドします。

## 使用方法
各ノードを起動するには、以下のコマンドを使用します。
```bash
ros2 run catchrobo_pkg <ノード名>
```

すべてのノードを一括で起動するには、以下のコマンドを使用します。
```bash
ros2 launch catchrobo_pkg catchrobo_launch.xml
```

### 一括ビルドと起動
以下のスクリプトを使用して、アンダーレイ設定、オーバーレイ設定、ビルド、そしてすべてのノードの起動を一括で行うことができます。

1. スクリプトに実行権限を付与します。
```bash
chmod +x src/catchrobo_pkg/setup_and_launch.sh
```

2. スクリプトを実行します。
```bash
./src/catchrobo_pkg/setup_and_launch.sh
```

## ライセンス
このパッケージは、適切なライセンスの下で提供されます。詳細は`LICENSE`ファイルを参照してください。