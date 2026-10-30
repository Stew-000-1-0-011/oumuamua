# Box Pickup Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** field.yamlに設定した5箱の初期x座標から、実形状で検査した箱回収を実行する。

**Architecture:** ROS非依存の計画と回収状態をC++コアへ分け、task_commanderは検証済みの段階を実行する。初期座標と表示は既存field.notes_xとcontrollerの可変個数機構を利用する。

**Tech Stack:** C++、ROS 2既存action/service、PyYAMLでの形状読込・起動設定、既存テスト構成。

**Spec:** `docs/superpowers/specs/2026-10-30-box-pickup-design.md`

## Global Constraints

- 5座標はfield.notes_x、m単位、箱中心、配列順がID。共通yと一辺長はfield.notes_y/note_size。
- 初期5値は旧設定の先頭5つの暫定値。実測値への変更を説明する。コードの個数固定はしない。
- 原形状の変換は `(x_base,y_base)=(-y_attachment,x_attachment)`。
- 非接触間隔20 mm、内壁片側padding2.5 mm、外周内面padding5 mm。対象箱との意図した吸着部接触だけを除外する。
- carryで移動→選択側clear→最終接近→pick/吸引→carry到達→退避。
- 吸引要求後の不明状態は再利用禁止。controller向けholding/picked状態と矛盾させない。
- controllerの座標入力機能、スラローム/棚タスクの作り替え、センサー追加、実機校正は対象外。

## Review Focus

- 任意順の5座標や不正/重複/重なり配置でも、箱番号が変わらず不正なら動かない（Task 1）。
- 端の箱・隣の箱・旋回中の外形で危険な経路を選ばない（Task 1）。
- 吸引要求が実際には届いたが応答不明の場合、箱と把持機構を空きに戻さない（Task 2）。
- 過去のlift_arrivedや古い自己位置を新しい到達として扱わない（Task 2）。
- carry失敗/退避失敗/取消後の記録がcontroller表示と一致する（Task 2）。

## Task 1: 回収計画と配置・形状検査

**Files:** Add `src/task_commander/include/task_commander/pickup.hpp`, `src/task_commander/src/pickup.cpp`, `src/task_commander/test/pickup_test.cpp`; extend geometry helpers if needed; update CMake test registration.

**Interfaces:** `PickupLayout` stores stable-ID x coordinates, common y and box size and validates the arrangement against the field. `PickupPlan` contains raised transit poses, lowered approach poses and raised retreat poses. `plan_pickup(...)` consumes current pose, selected tool offset, available boxes, upper/lower polygons and configured candidate yaw/transit/standoff/margins, returning a valid plan or a reason. No actuator or memory side effects occur while planning.

- [x] Write failing tests for stable IDs, invalid layouts, desired tool alignment, both tools, wall-end cases, neighboring boxes and position/yaw sweep violations. Verify target contact exemption is confined to the selected pad; other lower parts and upper chassis stay checked.
- [x] Implement exact polygon/rectangle checks and conservative interpolation validation matching `FollowPath` unnamed-goal shortest-yaw interpolation. Preserve the existing circle geometry interface for other tasks.
- [x] Generate preferred-yaw and offset-yaw candidates; check direct and outer-lane transit, outward +y standoff, final approach and retreat. Reject without side effects when none is safe.
- [x] Run standalone C++ pickup and existing geometry tests to success, using actual supplied shapes in integration fixtures.

## Task 2: 回収実行・状態・設定の接続

**Files:** Modify `src/task_commander/src/task_commander_node.cpp`, CMake and docs; add a testable pickup state helper and tests; modify `src/oumuamua_bringup/launch/bringup.launch.py`, `config/task_commander.yaml`, `config/field.yaml`, package dependencies if needed; add an isolated footprint-loading helper only if needed for testing.

**Interfaces:** Existing `RunTask` PICK IDs and `/task_commander/state` holding/picked syntax stay compatible. Geometry comes from shared footprint YAMLs transformed at startup, with resolved paths/point arrays supplied through bringup. Configuration adds preferred yaw alternatives and outer-lane position; tool offsets remain calibration inputs.

- [x] Write failing state/execution tests for holding rejection, stale feedback, suction-before/after failure, uncertain request result, carry failure, retreat failure and cancel at stage boundaries.
- [x] Integrate Task 1 preflight before any actuator commands. Execute all-upper transit, selected clear/approach, pick/suction/carry and retreat in order, checking fresh confirmations and failures.
- [x] Keep picked/held/uncertain accounting authoritative in task_commander; publish compatible unavailable states for failure recovery and require explicit reset after physical reconciliation.
- [x] Configure five initial x values and document shared-field restart, stable numbering and actual tool/lift calibration. Keep controller protocol unchanged; verify it receives five buttons and compatible state.
- [x] Run pickup/state/geometry tests plus Python slalom and C++ trajectory regressions. Run ROS integration/CI where available and state environment limits if unavailable.

## Task 3: 経路図・最終レビュー・リモート反映

- [x] Produce example pickup plan diagrams and record valid/rejected combinations for the configured five positions.
- [x] Independently review final changes and address material findings with regression tests.
- [x] Commit implementation and tests, preserve concurrent upstream changes, and publish a separate reviewable branch/PR based on the slalom changes.
