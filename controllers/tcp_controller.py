"""
Turin Robot TCP XML Controller
Protocol: TCP/IP, Port 8527, XML format, UTF-8
"""
import re
import socket
import threading
import time

XML_HEADER = '<?xml version="Turin.Robot.V2.0" encoding="UTF-8"?>'
_RE_DATA   = re.compile(r'<Data>(.*?)</Data>', re.DOTALL)
DEFAULT_PORT    = 8527
DEFAULT_TIMEOUT = 5.0
MOTION_TIMEOUT  = 60.0
RECV_BUFFER     = 65536


class TurinTCPController:
    def __init__(self, ip: str, port: int = DEFAULT_PORT, timeout: float = DEFAULT_TIMEOUT):
        self.ip      = ip
        self.port    = port
        self.timeout = timeout
        self._sock   = None
        self._cmd_counter = 0
        self._lock   = threading.Lock()

    # ── Connection ────────────────────────────────────────────────────────────

    def connect(self, username: str = "administrator", password: str = "12345678"):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.settimeout(self.timeout)
        self._sock.connect((self.ip, self.port))
        resp = self._send_cmd("Login", f'UserName="{username}" Password="{password}"')
        if not self._is_ok(resp):
            raise ConnectionError(f"Login failed: {self._get_data(resp)}")

    def disconnect(self):
        if self._sock:
            try:
                self._send_cmd("Logout")
            except Exception:
                pass
            self._sock.close()
            self._sock = None

    # ── Motion (blocking) ─────────────────────────────────────────────────────

    def move_linear(self, x: float, y: float, z: float,
                    rx: float = 0.0, ry: float = 0.0, rz: float = 0.0,
                    speed: int = 10):
        """Cartesian linear motion. Blocks until arm reaches target."""
        script = (
            f"MoveL({x:.3f},{y:.3f},{z:.3f},{rx:.3f},{ry:.3f},{rz:.3f},"
            f"0,0,0,0,0,0,0,0,0,0,0,0,{speed},0,1,0,01,00)"
        )
        return self._run_script(script)

    def move_joint(self, j1: float, j2: float, j3: float,
                   j4: float, j5: float, j6: float, speed: int = 10):
        """Joint-space motion. Blocks until arm reaches target."""
        script = (
            f"MoveJ(0,0,0,0,0,0,0,0,0,{j1:.3f},{j2:.3f},{j3:.3f},{j4:.3f},{j5:.3f},{j6:.3f},"
            f"0,0,0,{speed},0,1,0,01,00)"
        )
        return self._run_script(script)

    # ── Motion (non-blocking, for jog) ───────────────────────────────────────

    def move_linear_async(self, x: float, y: float, z: float,
                          rx: float = 0.0, ry: float = 0.0, rz: float = 0.0,
                          speed: int = 1):
        """
        Send MoveL without waiting for completion.
        Call stop() to interrupt the motion.
        Keep speed ≤ 1 to avoid motor overspeed error.
        """
        script = (
            f"MoveL({x:.3f},{y:.3f},{z:.3f},{rx:.3f},{ry:.3f},{rz:.3f},"
            f"0,0,0,0,0,0,0,0,0,0,0,0,{speed},0,1,0,01,00)"
        )
        param = 'UseInThread="false" IsGcode="false" MainFileName="tmp_jog.txt" StartLine="1" ExeLines="0" ExeLoops="1"'
        resp = self._send_cmd("MotionStart", param, data=script)
        if not self._is_ok(resp):
            raise RuntimeError(f"MoveL async failed: {self._get_data(resp)}")
        # Do NOT call _wait_stopped — returns immediately after ACK

    def move_joint_async(self, j1: float, j2: float, j3: float,
                         j4: float, j5: float, j6: float, speed: int = 1):
        """Send MoveJ without waiting for completion. Call stop() to interrupt."""
        script = (
            f"MoveJ(0,0,0,0,0,0,0,0,0,{j1:.3f},{j2:.3f},{j3:.3f},{j4:.3f},{j5:.3f},{j6:.3f},"
            f"0,0,0,{speed},0,1,0,01,00)"
        )
        param = 'UseInThread="false" IsGcode="false" MainFileName="tmp_jog.txt" StartLine="1" ExeLines="0" ExeLoops="1"'
        resp = self._send_cmd("MotionStart", param, data=script)
        if not self._is_ok(resp):
            raise RuntimeError(f"MoveJ async failed: {self._get_data(resp)}")

    def stop(self):
        """Interrupt current motion."""
        self._send_cmd("MotionStop")

    def jog_to_joint(self, signed_axis: int, target_deg: float, speed: int = 10):
        """Jog one joint to an absolute target angle and stop there automatically.
        signed_axis: +(1-6) if moving toward larger angle, -(1-6) if smaller."""
        param = (f'Mode="2" Motion="1" Operate="Start" Axis="{signed_axis}" '
                 f'Speed="{speed}" Offset="false" TarPos="{target_deg:.3f}"')
        resp = self._send_cmd("Jog", param)
        if not self._is_ok(resp):
            raise RuntimeError(f"Jog joint failed: {self._get_data(resp)}")

    def jog_start(self, motion: int, axis: int, speed: int = 3):
        """Start continuous jog. motion: 1=joint,2=base,3=tool,4=user. axis: ±1-6."""
        if motion == 1:
            # Joint mode requires TarPos; use ±360 as soft limit target
            tar = 360 if axis > 0 else -360
            param = (f'Mode="2" Motion="1" Operate="Start" Axis="{axis}" '
                     f'Speed="{speed}" Offset="false" TarPos="{tar}"')
        else:
            param = f'Mode="2" Motion="{motion}" Operate="Start" Axis="{axis}" Speed="{speed}"'
        resp = self._send_cmd("Jog", param)
        if not self._is_ok(resp):
            raise RuntimeError(f"Jog start failed: {self._get_data(resp)}")

    def jog_stop(self, motion: int, axis: int):
        """Stop continuous jog on the given axis."""
        param = f'Mode="2" Motion="{motion}" Operate="Stop" Axis="{abs(axis)}"'
        self._send_cmd("Jog", param)

    def home(self, speed: int = 10):
        param = f'Mode="1" Motion="1" Operate="Start" Speed="{speed}"'
        resp = self._send_cmd("Jog", param)
        if not self._is_ok(resp):
            raise RuntimeError(f"Home failed: {self._get_data(resp)}")
        self._wait_stopped()

    def clear_error(self):
        self._send_cmd("ClearRobotError")

    # ── IO ────────────────────────────────────────────────────────────────────

    def set_do(self, do_index: int, value: int):
        data = f"DO{do_index}={value}"
        resp = self._send_cmd("InputOutput", 'Comment="false"', data=data)
        if not self._is_ok(resp):
            raise RuntimeError(f"SetDO failed: {self._get_data(resp)}")

    def grasp(self, do_index: int = 0):
        self.set_do(do_index, 1)

    def release(self, do_index: int = 0):
        self.set_do(do_index, 0)

    # ── State ─────────────────────────────────────────────────────────────────

    def get_current_pos(self):
        """Returns (x, y, z, rx, ry, rz) in mm/deg using Tool=0 (flange/World frame)."""
        resp = self._send_cmd("GetCurrAllPos", 'Tool="0" User="0"')
        data = self._get_data(resp)
        pos  = self._parse_pos(data)
        world = pos.get("World", [])
        if len(world) >= 6:
            try:
                return tuple(float(v) for v in world[:6])
            except ValueError:
                pass
        return None

    def get_current_pos_tool1(self):
        """Returns (x, y, z, rx, ry, rz) in Tool=1 frame — used as MoveL jog targets."""
        resp = self._send_cmd("GetCurrAllPos", 'Tool="1" User="0"')
        data = self._get_data(resp)
        pos  = self._parse_pos(data)
        tool = pos.get("Tool", [])
        if len(tool) >= 6:
            try:
                return tuple(float(v) for v in tool[:6])
            except ValueError:
                pass
        return None

    def get_current_joints(self):
        """Returns (j1,j2,j3,j4,j5,j6) in degrees, or None on failure."""
        resp = self._send_cmd("GetCurrAllPos", 'Tool="1" User="0"')
        data = self._get_data(resp)
        pos   = self._parse_pos(data)
        joint = pos.get("Joint", [])
        if len(joint) >= 6:
            try:
                return tuple(float(v) for v in joint[:6])
            except ValueError:
                pass
        return None

    def get_status(self) -> str:
        resp = self._send_cmd("GetSystemRunningStatus")
        data = self._get_data(resp)
        for line in data.splitlines():
            if line.strip().lower().startswith("motionstatus:"):
                return line.split(":", 1)[1].strip()
        return "unknown"

    def get_full_status(self):
        """Returns (motion_status, has_error, error_msg)."""
        resp = self._send_cmd("GetSystemRunningStatus")
        data = self._get_data(resp)
        motion = "unknown"
        has_error = False
        error_msg = ""
        for line in data.splitlines():
            s = line.strip()
            if s.lower().startswith("motionstatus:"):
                motion = s.split(":", 1)[1].strip()
            key_lower = s.lower()
            if ("error" in key_lower or "alarm" in key_lower or "fault" in key_lower) and ":" in s:
                val = s.split(":", 1)[1].strip()
                if val not in ("0", "false", "False", "none", "None", "ok", "OK", ""):
                    has_error = True
                    error_msg = s
        return motion, has_error, error_msg

    def is_idle(self) -> bool:
        return self.get_status().lower() == "stopped"

    def heartbeat(self) -> bool:
        try:
            resp = self._send_cmd("LoopBack",
                                  f'currentMSecsSinceEpoch="{int(time.time()*1000)}" sendTimes="{self._cmd_counter}"')
            return resp is not None
        except Exception:
            return False

    # ── Internal ──────────────────────────────────────────────────────────────

    def _run_script(self, script: str):
        param = 'UseInThread="false" IsGcode="false" MainFileName="tmp_grasp.txt" StartLine="1" ExeLines="0" ExeLoops="1"'
        resp = self._send_cmd("MotionStart", param, data=script)
        if not self._is_ok(resp):
            raise RuntimeError(f"MotionStart failed: {self._get_data(resp)}")
        self._wait_stopped()

    def _wait_stopped(self, poll_interval: float = 0.1, timeout: float = 30.0):
        start = time.time()
        while time.time() - start < timeout:
            if self.is_idle():
                return
            time.sleep(poll_interval)
        raise TimeoutError("Arm motion did not complete within timeout.")

    def _build_xml(self, name: str, param_str: str = "", data: str = "") -> str:
        self._cmd_counter += 1
        param_tag = f"\n\t\t<Param {param_str}/>" if param_str else ""
        data_tag  = f"\n\t\t<Data>{data}</Data>" if data else ""
        return (
            f'{XML_HEADER}\n'
            f'<Bodys>\n'
            f'\t<Cmd Name="{name}" CmdCont="{self._cmd_counter}" Status="Send">'
            f'{param_tag}{data_tag}\n'
            f'\t</Cmd>\n'
            f'</Bodys>\n'
        )

    def _send_cmd(self, name: str, param_str: str = "", data: str = "") -> str:
        xml = self._build_xml(name, param_str, data)
        with self._lock:
            self._sock.sendall(xml.encode("utf-8"))
            return self._recv()

    def _recv(self) -> str:
        chunks = []
        while True:
            chunk = self._sock.recv(RECV_BUFFER)
            if not chunk:
                break
            chunks.append(chunk)
            if b"</Bodys>" in chunk:
                break
        return b"".join(chunks).decode("utf-8", errors="ignore")

    def _is_ok(self, xml_str: str) -> bool:
        if 'Status="Error"' in xml_str:
            return False
        return 'Status="Recv"' in xml_str

    def _get_data(self, xml_str: str) -> str:
        m = _RE_DATA.search(xml_str)
        return m.group(1).strip() if m else ""

    def _parse_pos(self, data: str) -> dict:
        result = {}
        for line in data.splitlines():
            line = line.strip()
            if ":" in line:
                key, val = line.split(":", 1)
                result[key.strip()] = [v.strip() for v in val.strip().split(",")]
        return result
