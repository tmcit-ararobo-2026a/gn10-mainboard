# メイン基板（STM32）自動モード & キーコンフィグ 仕様書

本仕様書は、メイン基板（STM32）における自動操縦・状態遷移制御、コントローラーキーコンフィグ、および ROS 2（Jetson）連携の仕様を定義するものです。

---

## 1. システム構成 & データフロー

```
[コントローラー (Teleop)]
       │ UDP: 10410 (teleop_t)
       ▼
[メイン基板 (main-board)] ─── UDP: 26574 (command_t, 50ms周期) ───▶ [Jetson (ROS 2)]
       │                                                                 │
       │                                                                 ├─ 自己位置推定: gn10-pointcloud-localization
       │                                                                 ├─ ナビゲーション: gn10-navigation
       │                                                                 └─ 追従/射出計算: opponent-tracker 等
       │                                                                 │
       ◀─── UDP: 39244 (operation_t, 周期/非同期) ───────────────────────┘
       │
[アクチュエータ群 (CAN / FDCAN / Solenoid)]
 (3輪オムニ足回り / ベルト直動射出 / 装填・バケツアーム / 空圧シリンダ)
```

詳細な PlantUML 図面は以下を参照してください：
- [system_data_flow.pu](system_data_flow.pu): システム間データ通信フロー
- [internal_arbitration.pu](internal_arbitration.pu): メイン基板内部の制御権限調停アーキテクチャ
- [state_machine.pu](state_machine.pu): 自動操縦モードの状態遷移図

---

## 2. 状態遷移 & 制御権限仕様 (State Machine)

メイン基板は Jetson から届く `operation_t.navigation_status` を同期し、以下の状態に応じて各アクチュエータの操作権限（手動 vs 自動 vs 安全ロック）を単一の調停モジュール（`ControlArbitrator`）で厳格に制御します。

### 状態ごとのアクチュエータ操作権限調停マッピング

| 状態 (`NavigationStatus`) | 水平・回転移動 (`vx, vy, vyaw`) | ベルト射出トリガー | ベルト威力 (速度) | バケツアーム・ハンド操作 | 備考 |
|---|---|---|---|---|---|
| **Standby** | **手動** (スティック) | **手動** (R1) | **手動** (右十字キー上下) | **手動** (左十字キー) | 電源投入後の初期手動モード |
| **Ready** | **手動** (スティック) | **手動** (R1) | **手動** (右十字キー上下) | **自動** (`op.bucket_arm_*`) | `gn10-pointcloud-localization` 成功後 |
| **Moving** | **自動** (`op.vel_*`) | **手動** (R1) | **自動** (`op.belt_launcher_speed`) | **不可** (出力ゼロ / 安全ロック) | `gn10-navigation` 走行中 |
| **Goal** | **手動** (スティック) | **手動** (R1) | **手動** (右十字キー上下) | **自動** (`op.bucket_arm_*`) | 目標地点到着、手動微調整・自動ハンド |
| **Tracking** | **水平: 手動** / **回転: 自動** (`op.vel_yaw`) | **手動** (R1) | **自動** (`op.belt_launcher_speed`) | **不可** (出力ゼロ / 安全ロック) | 移動バケツ追従中（並進は手動） |
| **Fail** | **手動** (スティック) | **手動** (R1) | **手動** (右十字キー上下) | **手動** (安全フォールバック) | 推定ロスト・通信断・エラー時 |

### 安全回路 (Fail-safe)
- **通信断検知**: Jetson からの `operation_t` 受信が 200ms 以上途絶えた場合、メイン基板は即座に `Fail` / `Sleep`（全手動操縦モード）へ強制移行します。
- **コントローラ断検知**: コントローラ通信が 100ms 途絶えた場合、全アクチュエータ出力を即時ゼロ停止します。

---

## 3. コントローラー キー割り当て仕様 (Key Configuration)

仕様書 `teleop_key_config.md` に準拠し、トグルスイッチで手動/自動モードを排他制御します。

### 3.1 モード共通操作

| 操作部位 | 機能 | 動作詳細 |
|---|---|---|
| **左トグルSW** (`left_toggle_sw`) | **モード切替** | `0`: 手動操縦モード / `1`: 自動操縦モード |
| **R1 スイッチ** (`right_trigger`) | **ベルト射出** | 押下エッジでベルト射出シーケンス起動 |
| **L1 スイッチ** (`left_trigger`) | **エアシリンダ射出** | 押下エッジで空圧ソレノイドを作動 |
| **右スティック押し込み** (`stick_push_right`) | **ベルト初期化** | ベルト原点復帰・再初期化コマンドを発行 |
| **左アナログスティック** | **水平移動 (X/Y)** | オムニホイール前後左右移動 |
| **右アナログスティック** | **旋回移動 (Yaw)** | 機体旋回 |

### 3.2 手動モード時 (`left_toggle_sw == 0`)

各アクチュエータを手動でダイレクト駆動します：
- **左十字キー 上 / 下**: バケツ用アーム昇降（上：上昇 / 下：降下）
- **左十字キー 左**: バケツ用アーム展開 / 収納（押下エッジごとにトグル切替）
- **左十字キー 右**: ハンド把持 / 解放（押下エッジごとにトグル切替）
- **右十字キー 上 / 下**: ベルト射出速度微調整（±0.5m/s）

展開・収納と把持・解放の状態は、ボタンを押している間ではなく、`false` から
`true` になった押下エッジで一度だけ反転する。したがって、ボタンを押し続けても
状態は毎周期反転せず、ボタンを離しても最後の状態を保持する。電源投入時は収納・
開放状態から開始する。

### 3.3 自動モード時 (`left_toggle_sw == 1`)

十字キー入力が動的に `TargetId` 送信（`command_t`）にオーバーライドされます：

| 入力キー | 動作種別 | 送信される `target_id` / コマンド |
|---|---|---|
| **左十字キー 上** | 押下エッジ | `TargetId::FixedBucket1` / `NavigationCommand::Move` |
| **左十字キー 右** | 押下エッジ | `TargetId::FixedBucket2` / `NavigationCommand::Move` |
| **左十字キー 下** | 押下エッジ | `TargetId::FixedBucket3` / `NavigationCommand::Move` |
| **左十字キー 左** | 押下エッジ | `TargetId::StartZone` / `NavigationCommand::Move` |
| **右十字キー 上** | 押下エッジ | `TargetId::Desk1` / `NavigationCommand::Move` |
| **右十字キー 下** | 押下エッジ | `TargetId::Desk2` / `NavigationCommand::Move` |
| **右十字キー 左** | 押下エッジ | `TargetId::Flag` / `NavigationCommand::Move` |
| **右十字キー 右** | **長押し中のみ** | `TargetId::MovingBucket` / `NavigationCommand::Move`（離すと `Stanby`） |
| **左スティック押し込み** | 押下エッジ | **キャンセル / 停止** (`NavigationCommand::Stanby`) |

---

## 4. ROS 2（Jetson）側への引き継ぎ内容

ROS 2側（`/home/gn10/ros2_ws`、ブランチ: `feat/robot-config-update-for-v5`）で実装が必要なシーケンス仕様です。

### 4.1 通信仕様
- **受信ポート `26574` (`command_t`)**: 50ms 周期でメイン基板から送信中。
  - `navigation_command`: `Sleep`, `Stanby`, `Move`
  - `target_id`: 移動対象のID
  - `localization`: `true`
- **送信ポート `39244` (`operation_t`)**: Jetsonからメイン基板へ周期送信。
  - `navigation_status`: 状態機械の現在ステータス（`Stanby`, `Ready`, `Moving`, `Tracking`, `Goal`, `Fail`）
  - `vel_x`, `vel_y`, `vel_yaw`: 自動走行・追従速度
  - `belt_launcher_speed`: 自動計算された射出初速
  - `bucket_arm_height`, `bucket_arm_width_extract`, `bucket_arm_hold`: アーム自動制御司令

### 4.2 ROS 2側ノード連携シーケンス要件

1. **自己位置推定連携 (`gn10-pointcloud-localization`)**:
   - 点群による自己位置推定が成功し、位置の信頼度・共分散が安定したら、状態を `Standby` ➔ `Ready` に遷移させて `operation_t.navigation_status = Ready` を返す。
2. **ナビゲーション連携 (`gn10-navigation`)**:
   - メイン基板から `navigation_command == Move` かつ固定ターゲット（バケツ・机・旗・スタートゾーン）を受信した場合：
     - `gn10-navigation` へ目標座標（ゴール）を発行し、状態を `Moving` へ遷移。
     - 到着して目標許容誤差以内に入ったら `Goal` へ遷移。
   - `navigation_command == Stanby`（キャンセル）を受信した場合：
     - ナビゲーションを中断し、状態を `Ready` に戻す。
3. **バケツ追従連携 (`opponent-tracker` 等)**:
   - `target_id == MovingBucket` かつ `navigation_command == Move` を受信中：
     - 状態を `Tracking` に遷移。
     - 移動バケツの推定位置に向けて射線が合うよう `vel_yaw` を計算して送信。
     - ボタンが離されて `Stanby` を受信したら追従を終了し `Ready` に戻す。
4. **異常検知 (`Fail`)**:
   - 自己位置ロスト、危険領域侵入、センサー通信異常、バケツ見失いタイムアウト時は `Fail` を送信し、メイン基板側に手動制御権を委譲する。
