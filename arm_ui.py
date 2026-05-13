"""
Turin Single-Arm Controller UI
- Jog XYZ (TCP frame, motion=3)
- Jog Joints J1-J6 (motion=1)
- HOME, SAVE POS, GO POS
- Alarm indicator + CLEAR ERR
- DO9 toggle (AUTO / MANUAL)
"""
import threading
import time
import yaml
import cv2
import numpy as np
import sys
import os

sys.path.insert(0, os.path.dirname(__file__))
from controllers.tcp_controller import TurinTCPController

CONFIG_PATH = os.path.join(os.path.dirname(__file__), "config", "settings.yaml")

WIN_W = 420
WIN_H = 720
WIN_NAME = "Arm Controller"

JOG_SPEED  = 3
BTN_H      = 32
BTN_W_HALF = 60   # half-width jog button
BTN_W_FULL = 130  # full-width action button

# Colours (BGR)
C_BG       = (30, 30, 30)
C_BTN      = (70, 70, 70)
C_BTN_HOV  = (100, 100, 100)
C_BTN_ACT  = (50, 120, 200)
C_BTN_ON   = (40, 160, 80)
C_BTN_OFF  = (160, 60, 40)
C_TEXT     = (220, 220, 220)
C_LABEL    = (160, 160, 160)
C_OK       = (60, 180, 60)
C_ERR      = (60, 60, 200)
C_SECTION  = (55, 55, 55)

FONT       = cv2.FONT_HERSHEY_SIMPLEX
FS_SM      = 0.42
FS_MD      = 0.50
FS_LG      = 0.60

# ── Jog axis definitions ────────────────────────────────────────────────────
# (label, motion, axis)
JOG_XYZ = [
    ("+X", 3,  1), ("-X", 3, -1),
    ("+Y", 3,  2), ("-Y", 3, -2),
    ("+Z", 3,  3), ("-Z", 3, -3),
]
JOG_JOINT = [
    ("+J1", 1,  1), ("-J1", 1, -1),
    ("+J2", 1,  2), ("-J2", 1, -2),
    ("+J3", 1,  3), ("-J3", 1, -3),
    ("+J4", 1,  4), ("-J4", 1, -4),
    ("+J5", 1,  5), ("-J5", 1, -5),
    ("+J6", 1,  6), ("-J6", 1, -6),
]


def _draw_btn(img, rect, label, color=C_BTN, text_color=C_TEXT, fs=FS_SM):
    x, y, w, h = rect
    cv2.rectangle(img, (x, y), (x+w, y+h), color, -1)
    cv2.rectangle(img, (x, y), (x+w, y+h), (90, 90, 90), 1)
    tw, th = cv2.getTextSize(label, FONT, fs, 1)[0]
    tx = x + (w - tw) // 2
    ty = y + (h + th) // 2
    cv2.putText(img, label, (tx, ty), FONT, fs, text_color, 1, cv2.LINE_AA)


def _in_rect(px, py, rect):
    x, y, w, h = rect
    return x <= px < x+w and y <= py < y+h


class ArmUI:
    def __init__(self):
        with open(CONFIG_PATH) as f:
            self._cfg = yaml.safe_load(f)
        arm_cfg = self._cfg["arm"]

        self._arm = TurinTCPController(arm_cfg["ip"], arm_cfg.get("port", 8527))
        self._home_joints = arm_cfg.get("home_joints")
        self._saved_pos   = arm_cfg.get("saved_pos")  # [x,y,z,rx,ry,rz] or None

        self._robot_ok     = True
        self._robot_status = "unknown"
        self._tcp_pos      = None   # (x,y,z,rx,ry,rz)
        self._joints       = None   # (j1..j6)

        self._do9_state    = False  # False=manual, True=auto
        self._do9_lock     = threading.Lock()

        self._jog_request  = None   # (motion, axis) or None
        self._jog_current  = None
        self._jog_lock     = threading.Lock()

        self._busy         = False
        self._busy_msg     = ""
        self._connected    = False
        self._quit         = False

        # Build button rects (computed once)
        self._build_rects()

        # Connect in background so UI opens immediately
        threading.Thread(target=self._connect_loop, daemon=True).start()
        threading.Thread(target=self._jog_loop,  daemon=True).start()

    # ── Layout ─────────────────────────────────────────────────────────────────

    def _build_rects(self):
        """Pre-compute all button rects."""
        mx = 12   # margin x
        y  = 10

        # alarm + status bar — just drawn, no clickable rect needed
        y += 30   # status row height

        # ── TCP position ──
        y += 18   # section label
        self._r_tcp_area = (mx, y, WIN_W - mx*2, 44)
        y += 50

        # ── Joint position ──
        y += 18
        self._r_jnt_area = (mx, y, WIN_W - mx*2, 44)
        y += 50

        # ── DO9 toggle ──
        y += 8
        self._r_do9 = (mx, y, BTN_W_FULL, BTN_H)
        y += BTN_H + 10

        # ── JOG XYZ ──
        y += 18   # section label
        self._r_xyz = []
        bw = BTN_W_HALF
        gap = 6
        cols = [(mx + i*(bw+gap)) for i in range(3)]
        for i, (lbl, mot, ax) in enumerate(JOG_XYZ):
            col = i // 2
            row = i %  2
            rx = cols[col]
            ry = y + row * (BTN_H + gap)
            self._r_xyz.append((rx, ry, bw, BTN_H))
        y += 2*(BTN_H + gap) + 6

        # ── JOG Joint ──
        y += 18
        self._r_jnt = []
        cols2 = [(mx + i*(bw+gap)) for i in range(6)]
        for i, (lbl, mot, ax) in enumerate(JOG_JOINT):
            col = i // 2
            row = i %  2
            rx = cols2[col]
            ry = y + row * (BTN_H + gap)
            self._r_jnt.append((rx, ry, bw, BTN_H))
        y += 2*(BTN_H + gap) + 10

        # ── Action buttons ──
        y += 4
        abw = 90
        agap = 8
        self._r_home     = (mx,               y, abw, BTN_H)
        self._r_save_pos = (mx + (abw+agap),   y, abw, BTN_H)
        self._r_go_pos   = (mx + 2*(abw+agap), y, abw, BTN_H)
        y += BTN_H + 8

        self._r_clear_err = (mx, y, abw, BTN_H)
        self._r_quit      = (WIN_W - mx - abw, y, abw, BTN_H)
        y += BTN_H + 8

        self._layout_h = y

    # ── Connect loop ───────────────────────────────────────────────────────────

    def _connect_loop(self):
        ip = self._cfg["arm"]["ip"]
        while not self._connected:
            try:
                print(f"[arm] Connecting to {ip}...")
                self._robot_status = "connecting..."
                self._arm.connect()
                self._connected    = True
                self._robot_status = "connected"
                print("[arm] Connected.")
                threading.Thread(target=self._poll_loop, daemon=True).start()
            except Exception as e:
                print(f"[arm] Connect failed: {e}  retrying in 5s...")
                self._robot_status = "disconnected"
                self._robot_ok     = False
                time.sleep(5)

    # ── Poll loop ──────────────────────────────────────────────────────────────

    def _poll_loop(self):
        while True:
            time.sleep(0.5)
            if not self._connected:
                continue
            with self._jog_lock:
                jogging = self._jog_current is not None
            if jogging:
                continue
            try:
                motion, has_err, err_msg = self._arm.get_full_status()
                self._robot_status = motion
                self._robot_ok     = not has_err
                pos = self._arm.get_current_pos()
                if pos:
                    self._tcp_pos = pos
                jnts = self._arm.get_current_joints()
                if jnts:
                    self._joints = jnts
            except Exception as e:
                self._robot_ok     = False
                self._robot_status = "comm error"

    # ── Jog loop ───────────────────────────────────────────────────────────────

    def _jog_loop(self):
        while True:
            time.sleep(0.04)
            if not self._connected:
                continue
            with self._jog_lock:
                req = self._jog_request
                cur = self._jog_current
            if req == cur:
                continue
            try:
                if cur is not None:
                    self._arm.jog_stop(cur[0], cur[1])
                if req is not None:
                    self._arm.jog_start(req[0], req[1], JOG_SPEED)
            except Exception as e:
                print(f"[jog] {e}")
                req = None
            with self._jog_lock:
                self._jog_current = req

    # ── DO9 ────────────────────────────────────────────────────────────────────

    def _toggle_do9(self):
        with self._do9_lock:
            new_state = not self._do9_state
        try:
            self._arm.set_do(9, 1 if new_state else 0)
            with self._do9_lock:
                self._do9_state = new_state
            print(f"[do9] {'AUTO' if new_state else 'MANUAL'}")
        except Exception as e:
            print(f"[do9] ERROR: {e}")

    # ── Draw ───────────────────────────────────────────────────────────────────

    def _draw(self):
        img = np.full((WIN_H, WIN_W, 3), C_BG, dtype=np.uint8)
        mx = 12
        y  = 10

        # Status bar
        dot_color = C_OK if self._robot_ok else C_ERR
        cv2.circle(img, (mx + 8, y + 10), 7, dot_color, -1)
        status_text = self._robot_status
        cv2.putText(img, status_text, (mx + 22, y + 15), FONT, FS_MD, C_TEXT, 1, cv2.LINE_AA)
        y += 30

        # ── TCP pos ──
        cv2.putText(img, "TCP Position (mm / deg)", (mx, y + 13), FONT, FS_SM, C_LABEL, 1, cv2.LINE_AA)
        y += 18
        cv2.rectangle(img, (mx, y), (mx + WIN_W - mx*2, y + 44), C_SECTION, -1)
        if self._tcp_pos:
            names = ["X", "Y", "Z", "Rx", "Ry", "Rz"]
            for i, (n, v) in enumerate(zip(names, self._tcp_pos)):
                col_x = mx + 4 + i * 65
                cv2.putText(img, n, (col_x, y + 16), FONT, FS_SM, C_LABEL, 1, cv2.LINE_AA)
                cv2.putText(img, f"{v:.1f}", (col_x, y + 36), FONT, FS_SM, C_TEXT, 1, cv2.LINE_AA)
        else:
            cv2.putText(img, "--", (mx + 8, y + 28), FONT, FS_MD, C_LABEL, 1, cv2.LINE_AA)
        y += 50

        # ── Joint pos ──
        cv2.putText(img, "Joint Position (deg)", (mx, y + 13), FONT, FS_SM, C_LABEL, 1, cv2.LINE_AA)
        y += 18
        cv2.rectangle(img, (mx, y), (mx + WIN_W - mx*2, y + 44), C_SECTION, -1)
        if self._joints:
            names = ["J1", "J2", "J3", "J4", "J5", "J6"]
            for i, (n, v) in enumerate(zip(names, self._joints)):
                col_x = mx + 4 + i * 65
                cv2.putText(img, n, (col_x, y + 16), FONT, FS_SM, C_LABEL, 1, cv2.LINE_AA)
                cv2.putText(img, f"{v:.1f}", (col_x, y + 36), FONT, FS_SM, C_TEXT, 1, cv2.LINE_AA)
        else:
            cv2.putText(img, "--", (mx + 8, y + 28), FONT, FS_MD, C_LABEL, 1, cv2.LINE_AA)
        y += 50

        # ── DO9 toggle ──
        y += 8
        with self._do9_lock:
            auto = self._do9_state
        do9_color = C_BTN_ON if auto else C_BTN_OFF
        do9_label = "DO9: AUTO" if auto else "DO9: MANUAL"
        _draw_btn(img, self._r_do9, do9_label, color=do9_color)
        y += BTN_H + 10

        # ── JOG XYZ ──
        cv2.putText(img, "Jog XYZ (TCP frame)", (mx, y + 13), FONT, FS_SM, C_LABEL, 1, cv2.LINE_AA)
        y += 18
        with self._jog_lock:
            cur = self._jog_current
        for i, ((lbl, mot, ax), rect) in enumerate(zip(JOG_XYZ, self._r_xyz)):
            active = cur is not None and cur == (mot, ax)
            color  = C_BTN_ACT if active else C_BTN
            _draw_btn(img, rect, lbl, color=color)
        y += 2*(BTN_H + 6) + 6

        # ── JOG Joint ──
        cv2.putText(img, "Jog Joints", (mx, y + 13), FONT, FS_SM, C_LABEL, 1, cv2.LINE_AA)
        y += 18
        for i, ((lbl, mot, ax), rect) in enumerate(zip(JOG_JOINT, self._r_jnt)):
            active = cur is not None and cur == (mot, ax)
            color  = C_BTN_ACT if active else C_BTN
            _draw_btn(img, rect, lbl, color=color)
        y += 2*(BTN_H + 6) + 10

        # ── Action buttons ──
        y += 4
        _draw_btn(img, self._r_home,     "HOME",     color=C_BTN)
        save_col = C_BTN_ON if self._saved_pos else C_BTN
        _draw_btn(img, self._r_save_pos, "SAVE POS", color=save_col)
        go_col   = C_BTN_ACT if self._saved_pos else (50, 50, 50)
        _draw_btn(img, self._r_go_pos,   "GO POS",   color=go_col)
        y += BTN_H + 8

        _draw_btn(img, self._r_clear_err, "CLEAR ERR", color=(140, 60, 60))
        _draw_btn(img, self._r_quit,      "QUIT",      color=(100, 40, 40))

        if self._busy:
            overlay = img.copy()
            cv2.rectangle(overlay, (0, 0), (WIN_W, WIN_H), (0, 0, 0), -1)
            cv2.addWeighted(overlay, 0.45, img, 0.55, 0, img)
            tw, th = cv2.getTextSize(self._busy_msg, FONT, FS_LG, 1)[0]
            cv2.putText(img, self._busy_msg,
                        ((WIN_W-tw)//2, (WIN_H+th)//2), FONT, FS_LG, C_TEXT, 1, cv2.LINE_AA)

        return img

    # ── Mouse ─────────────────────────────────────────────────────────────────

    def _on_mouse(self, event, px, py, flags, param):
        # LBUTTONUP anywhere → stop jog
        if event == cv2.EVENT_LBUTTONUP:
            with self._jog_lock:
                self._jog_request = None
            return

        if event != cv2.EVENT_LBUTTONDOWN:
            return

        # QUIT — always available
        if _in_rect(px, py, self._r_quit):
            self._quit = True
            return

        # CLEAR ERR — always available
        if _in_rect(px, py, self._r_clear_err):
            threading.Thread(target=self._do_clear_err, daemon=True).start()
            return

        # DO9 toggle
        if _in_rect(px, py, self._r_do9):
            threading.Thread(target=self._toggle_do9, daemon=True).start()
            return

        if self._busy:
            return

        # Jog XYZ
        for (lbl, mot, ax), rect in zip(JOG_XYZ, self._r_xyz):
            if _in_rect(px, py, rect):
                with self._jog_lock:
                    self._jog_request = (mot, ax)
                return

        # Jog Joint
        for (lbl, mot, ax), rect in zip(JOG_JOINT, self._r_jnt):
            if _in_rect(px, py, rect):
                with self._jog_lock:
                    self._jog_request = (mot, ax)
                return

        # HOME
        if _in_rect(px, py, self._r_home):
            threading.Thread(target=self._do_home, daemon=True).start()
            return

        # SAVE POS
        if _in_rect(px, py, self._r_save_pos):
            threading.Thread(target=self._do_save_pos, daemon=True).start()
            return

        # GO POS
        if self._saved_pos and _in_rect(px, py, self._r_go_pos):
            threading.Thread(target=self._do_go_pos, daemon=True).start()
            return

    # ── Actions ────────────────────────────────────────────────────────────────

    def _do_home(self):
        if not self._home_joints:
            print("[home] No home_joints in config.")
            return
        self._busy = True
        self._busy_msg = "Homing..."
        try:
            j = self._home_joints
            print(f"[home] MoveL to home joints: {j}")
            self._arm.move_joint(*j, speed=10)
            print("[home] Done.")
        except Exception as e:
            print(f"[home] ERROR: {e}")
        finally:
            self._busy = False

    def _do_save_pos(self):
        try:
            pos = self._arm.get_current_pos()
            if pos:
                self._saved_pos = list(pos)
                # Persist to settings.yaml
                self._cfg["arm"]["saved_pos"] = self._saved_pos
                with open(CONFIG_PATH, "w") as f:
                    yaml.dump(self._cfg, f, default_flow_style=False, allow_unicode=True)
                print(f"[save_pos] Saved: {[f'{v:.2f}' for v in pos]}")
        except Exception as e:
            print(f"[save_pos] ERROR: {e}")

    def _do_go_pos(self):
        if not self._saved_pos:
            return
        self._busy = True
        self._busy_msg = "Moving to saved pos..."
        try:
            x, y, z, rx, ry, rz = self._saved_pos
            print(f"[go_pos] MoveL {x:.2f},{y:.2f},{z:.2f},{rx:.2f},{ry:.2f},{rz:.2f}")
            self._arm.move_linear(x, y, z, rx, ry, rz, speed=10)
            print("[go_pos] Done.")
        except Exception as e:
            print(f"[go_pos] ERROR: {e}")
        finally:
            self._busy = False

    def _do_clear_err(self):
        try:
            self._arm.clear_error()
            self._robot_ok = True
            print("[clear_err] Cleared.")
        except Exception as e:
            print(f"[clear_err] ERROR: {e}")

    # ── Run ────────────────────────────────────────────────────────────────────

    def run(self):
        cv2.namedWindow(WIN_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WIN_NAME, WIN_W, WIN_H)
        cv2.setMouseCallback(WIN_NAME, self._on_mouse)
        print("[ui] Window open. Click jog buttons; release mouse to stop.")

        while True:
            img = self._draw()
            cv2.imshow(WIN_NAME, img)
            key = cv2.waitKey(30) & 0xFF
            if key == 27 or self._quit:
                break
            try:
                if cv2.getWindowProperty(WIN_NAME, cv2.WND_PROP_VISIBLE) < 1:
                    break
            except cv2.error:
                break

        # Stop any active jog before exit
        with self._jog_lock:
            cur = self._jog_current
            self._jog_request = None
        if cur:
            try:
                self._arm.jog_stop(cur[0], cur[1])
            except Exception:
                pass
        self._arm.disconnect()
        cv2.destroyAllWindows()
        print("[ui] Exited.")


if __name__ == "__main__":
    ui = ArmUI()
    ui.run()
