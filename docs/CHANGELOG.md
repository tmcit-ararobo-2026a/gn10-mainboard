# 変更履歴 (ChangeLog) - 自動モード状態遷移 & キーコンフィグ実装

本ドキュメントは、ブランチ `refactor/auto-mode-state-machine` におけるメイン基板ファームウェアの変更履歴です。

---

## [Unreleased] - 2026-10-12

### 概要
自動モード状態遷移（PlantUML仕様）およびキーコンフィグ（`teleop_key_config.md`）の実装、ならびに保守性向上のための調停アーキテクチャへのリファクタリングを実施。
- 手動モードのアーム展開・収納およびハンド把持・解放を、押下エッジごとに一度だけ反転するトグル動作へ修正。

### 追加 (Added)
- **`app/include/app/control_arbitrator.hpp` / `app/src/control_arbitrator.cpp`**:
  - `ControlArbitrator` クラスを新規追加。
  - 現在の `NavigationStatus`（Standby, Ready, Moving, Tracking, Goal, Fail）に応じて、各アクチュエータ（オムニ足回り、ベルト直動射出、バケツアーム・ハンド、空圧シリンダ）の操作権限（手動 vs 自動 vs 安全ロック）を一元調停するロジックを実装。
- **`app/include/app/robot_state_machine.hpp` / `app/src/robot_state_machine.cpp`**:
  - `RobotStateMachine` クラスを新規追加。
  - コントローラーのトグルスイッチ（手動/自動）状態、十字キーによる動的な `TargetId` / `navigation_command` 生成を管理。
  - Jetsonとの `NavigationStatus` 同期および、200ms通信断時の手動フェールセーフ回路を実装。
- **ドキュメント (`docs/`)**:
  - `docs/system_data_flow.pu`: システム全体データフロー（PlantUML）
  - `docs/internal_arbitration.pu`: メイン基板内部の調停構造図（PlantUML）
  - `docs/state_machine.pu`: 自動操縦モード状態遷移図（PlantUML）
  - `docs/auto_mode_specification.md`: 自動モード & キーコンフィグ総合仕様書（ROS 2引き継ぎ仕様含む）
  - `docs/CHANGELOG.md`: 本変更履歴ドキュメント

### 変更 (Changed)
- **`app/src/app.cpp`**:
  - `command_robot_drivers()` をリファクタリング。アクチュエータ直接操作から `ControlArbitrator` を経由した調停出力方式へ移行。
  - `teleop_key_config.md` に基づき、L1/R1トリガー射出（ベルト/空圧）、手動アーム展開/把持トグルスイッチを反映。
  - `loop()` 周期内で `ether.send_command_data()` を 50ms 周期で定期送信するように追加。
- **`app/src/bucket_arm_controller.cpp`**:
  - `width_motor_output()` で戻り値が抜けていたコンパイル警告を修正。
  - 展開・収納の前回入力を保存し、押下中の周期ごとの誤反転を防止。
  - ハンド把持・解放も押下エッジによるトグル動作に統一。
- **`app/CMakeLists.txt`**:
  - `app` 静的ライブラリのビルド対象に `robot_state_machine.cpp` と `control_arbitrator.cpp` を追加。
