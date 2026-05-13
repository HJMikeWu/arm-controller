# arm-controller

基於 OpenCV 的 Turin TCR-010 單臂控制 UI。

## 功能

- **Jog XYZ** — TCP 工具座標系（+X/-X、+Y/-Y、+Z/-Z），按住移動，放開停止
- **Jog 關節** — J1–J6 關節空間 jog
- **HOME** — 移動至 config 設定的 home 關節角度
- **SAVE POS / GO POS** — 記錄當前 TCP 位置，並可移回該位置
- **DO9 切換** — 數位輸出控制手動 / 自動模式切換
- **警報指示燈 + CLEAR ERR** — 顯示機械臂狀態，一鍵清除錯誤
- **QUIT 按鈕** — 正常結束程式
- 背景 TCP 連線，自動重試（視窗立即開啟不阻塞）

## 硬體

| 裝置 | IP | 備註 |
|------|-----|------|
| Turin 機械臂 | 192.168.0.103 | TCP port 8527 |

## 安裝

**Linux（MIC-733）：**
```bash
bash setup.sh
```

**macOS：**
```bash
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
```

## 執行

**Linux：**
```bash
.venv/bin/python3 -u arm_ui.py
```

**macOS：**
```bash
DYLD_LIBRARY_PATH=/opt/homebrew/Cellar/expat/2.8.0/lib .venv/bin/python3 -u arm_ui.py
```

## 設定

編輯 `config/settings.yaml` 設定 IP、port 與 home 關節角度：

```yaml
arm:
  ip: "192.168.0.103"
  port: 8527
  home_joints: [-23.038, 86.39, -40.894, 0.042, 45.525, 37.733]
  saved_pos: null  # 由 SAVE POS 按鈕自動寫入
```

## Turin 機械臂注意事項

- 只有 `MoveL`（笛卡爾直線）可用，`MoveJ` 在 MotionControlMode=48 下無效。
- `jog_stop` 必須使用 `abs(axis)`，Turin 不接受負軸號的 Stop 指令。
- 狀態比較須不分大小寫（機械臂回傳 `"Stopped"` 或 `"stopped"`）。
- DO9=1 → 自動模式，DO9=0 → 手動模式。
