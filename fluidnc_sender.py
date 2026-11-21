#!/usr/bin/env python3
"""
FluidNC WebSocket & HTTP G-code Controller and Streamer.

Direct streaming and communication tool for FluidNC / MKS-DLC32 CNC controllers.
Supports:
  - Real-time WebSocket G-code streaming with flow control and progress bar
  - Machine status query (MPos, WPos, State, Alarms, Spindle, Feed)
  - SD card detection and HTTP multipart file upload
  - Quick machine commands: Unlock ($X), Home ($H), Zero Work, Soft Reset, Feed Hold, Resume
"""

import sys
import os
import time
import argparse
import socket
import base64
import urllib.request
import urllib.parse
import json
import re

DEFAULT_IP = "192.168.8.109"
DEFAULT_PORT = 80


class FluidNCClient:
    def __init__(self, host=DEFAULT_IP, port=DEFAULT_PORT, timeout=4.0):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.sock = None
        self.connected = False
        self.last_status = {}

    def connect(self):
        """Establish WebSocket connection with FluidNC using arduino subprotocol."""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(self.timeout)
            self.sock.connect((self.host, self.port))

            key = base64.b64encode(os.urandom(16)).decode("utf-8")
            req = (
                f"GET /?independent_session=1 HTTP/1.1\r\n"
                f"Host: {self.host}:{self.port}\r\n"
                f"Upgrade: websocket\r\n"
                f"Connection: Upgrade\r\n"
                f"Sec-WebSocket-Key: {key}\r\n"
                f"Sec-WebSocket-Protocol: arduino\r\n"
                f"Sec-WebSocket-Version: 13\r\n\r\n"
            )
            self.sock.sendall(req.encode("utf-8"))
            resp = self.sock.recv(4096).decode("latin1", errors="ignore")
            if "101 Switching Protocols" not in resp:
                raise ConnectionError(f"WebSocket upgrade rejected by {self.host}: {resp[:100]}")
            self.connected = True
            time.sleep(0.05)
            # Flush initial welcome banner
            self._read_available()
            return True
        except Exception as e:
            self.connected = False
            raise ConnectionError(f"Failed to connect to FluidNC at {self.host}:{self.port} -> {e}")

    def close(self):
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None
        self.connected = False

    def _send_frame(self, payload: bytes, opcode: int = 0x01):
        """Send masked WebSocket frame to server (opcode 1=text, 2=binary)."""
        if not self.sock:
            raise ConnectionError("Not connected")
        mask = os.urandom(4)
        length = len(payload)
        header = bytearray([0x80 | (opcode & 0x0F)])
        if length <= 125:
            header.append(0x80 | length)
        elif length <= 65535:
            header.append(0x80 | 126)
            header.extend(length.to_bytes(2, "big"))
        else:
            header.append(0x80 | 127)
            header.extend(length.to_bytes(8, "big"))

        masked_payload = bytearray(b ^ mask[i % 4] for i, b in enumerate(payload))
        self.sock.sendall(header + mask + masked_payload)

    def _read_available(self, timeout=0.1):
        """Read all available WebSocket frames within timeout."""
        frames = []
        if not self.sock:
            return frames
        orig_timeout = self.sock.gettimeout()
        self.sock.settimeout(timeout)
        try:
            while True:
                data = self.sock.recv(4096)
                if not data:
                    break
                idx = 0
                while idx < len(data):
                    if idx + 2 > len(data):
                        break
                    op = data[idx] & 0x0F
                    l = data[idx + 1] & 0x7F
                    idx += 2
                    if l == 126:
                        if idx + 2 > len(data):
                            break
                        l = int.from_bytes(data[idx:idx + 2], "big")
                        idx += 2
                    elif l == 127:
                        if idx + 8 > len(data):
                            break
                        l = int.from_bytes(data[idx:idx + 8], "big")
                        idx += 8
                    payload = data[idx:idx + l]
                    idx += l
                    text = payload.decode("utf-8", errors="ignore")
                    frames.append((op, text))
        except socket.timeout:
            pass
        except Exception:
            pass
        finally:
            if self.sock:
                self.sock.settimeout(orig_timeout)
        return frames

    def send_realtime(self, char: str):
        """Send 1-byte realtime command (?, !, ~, Ctrl+X)."""
        b = ord(char[0])
        # Binary frame opcode 2
        self._send_frame(bytes([b]), opcode=0x02)

    def send_line(self, line: str):
        """Send G-code or $-command line."""
        clean = line.strip() + "\n"
        self._send_frame(clean.encode("utf-8"), opcode=0x01)

    def query_status(self):
        """Send '?' and parse Grbl status packet."""
        self.send_realtime("?")
        time.sleep(0.08)
        frames = self._read_available(timeout=0.3)
        for _, text in frames:
            for line in text.split("\n"):
                line = line.strip()
                if line.startswith("<") and line.endswith(">"):
                    self._parse_status(line)
        return self.last_status

    def _parse_status(self, raw_str: str):
        body = raw_str[1:-1]
        parts = body.split("|")
        status = {"state": parts[0], "raw": raw_str}
        for p in parts[1:]:
            if ":" in p:
                k, v = p.split(":", 1)
                status[k] = v
        self.last_status = status
        return status

    def check_sd_status(self):
        """Query HTTP endpoint to check SD card mounting."""
        try:
            url = f"http://{self.host}:{self.port}/upload?path=/"
            req = urllib.request.Request(url, headers={"User-Agent": "FluidNCClient/1.0"})
            with urllib.request.urlopen(req, timeout=3.0) as resp:
                data = json.loads(resp.read().decode("utf-8"))
                if data.get("status") == "No SD card":
                    return False, "No micro-SD card detected in board TF/SD slot"
                return True, f"SD card mounted ({data.get('total', 'Unknown')} total)"
        except Exception as e:
            return False, str(e)

    def upload_file(self, filepath: str, target: str = "sd", run_after: bool = False):
        """Upload a G-code file to FluidNC SD card (/upload) or LocalFS (/files) using FigUI's multipart format."""
        import uuid
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"G-code file not found: {filepath}")

        filename = os.path.basename(filepath)
        remote_path = f"/{filename}"
        with open(filepath, "rb") as f:
            content = f.read()

        endpoint = f"http://{self.host}:{self.port}/upload" if target == "sd" else f"http://{self.host}:{self.port}/files"
        boundary = "----FluidNC" + uuid.uuid4().hex[:16]
        body = bytearray()
        body.extend(f"--{boundary}\r\n".encode("utf-8"))
        body.extend(b'Content-Disposition: form-data; name="path"\r\n\r\n')
        body.extend(b"/\r\n")
        body.extend(f"--{boundary}\r\n".encode("utf-8"))
        body.extend(f'Content-Disposition: form-data; name="{remote_path}S"\r\n\r\n'.encode("utf-8"))
        body.extend(f"{len(content)}\r\n".encode("utf-8"))
        body.extend(f"--{boundary}\r\n".encode("utf-8"))
        body.extend(f'Content-Disposition: form-data; name="myfile[]"; filename="{remote_path}"\r\n'.encode("utf-8"))
        body.extend(b"Content-Type: application/octet-stream\r\n\r\n")
        body.extend(content)
        body.extend(f"\r\n--{boundary}--\r\n".encode("utf-8"))

        req = urllib.request.Request(
            endpoint,
            data=bytes(body),
            headers={"Content-Type": f"multipart/form-data; boundary={boundary}", "User-Agent": "FluidNCClient/1.0"},
            method="POST"
        )
        with urllib.request.urlopen(req, timeout=20.0) as resp:
            resp_text = resp.read().decode("utf-8", errors="replace")
            if "No SD card" in resp_text:
                raise RuntimeError("Upload rejected: No micro-SD card mounted in MKS-DLC32 TF slot.")
            print(f"[✓] Uploaded {filename} ({len(content)} bytes) to {'/sd/' if target == 'sd' else '/files/'}")

        if run_after:
            run_cmd = f"$SD/Run={remote_path}" if target == "sd" else f"$LocalFS/Run={remote_path}"
            print(f"[*] Triggering job execution: {run_cmd}")
            self.send_line(run_cmd)
            time.sleep(0.25)
            for _, text in self._read_available(timeout=0.5):
                if text.strip():
                    print("    FluidNC:", text.strip())

    def stream_gcode(self, filepath: str, on_progress=None):
        """
        Stream a G-code file over WebSocket using GRBL/FluidNC 127-byte character-counting buffer.
        """
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"G-code file not found: {filepath}")

        with open(filepath, "r", encoding="utf-8") as f:
            raw_lines = f.readlines()

        # Pre-filter executable G-code lines
        exec_lines = []
        for raw in raw_lines:
            clean = raw.strip()
            if not clean or clean.startswith(";") or clean.startswith("("):
                continue
            exec_lines.append(clean)

        total_lines = len(exec_lines)
        if total_lines == 0:
            print("[!] No executable G-code lines found.")
            return

        st = self.query_status()
        state = st.get("state", "Unknown")
        if "Alarm" in state:
            raise RuntimeError(f"Cannot stream: Machine is in ALARM state ({state}). Run --unlock ($X) or --home ($H) first.")

        print(f"[*] Starting G-code stream: {os.path.basename(filepath)}")
        print(f"    Executable lines: {total_lines} | Machine state: {state}")

        RX_BUFFER_SIZE = 127
        in_flight_lens = []
        sent_count = 0
        acked_count = 0
        start_time = time.time()

        def process_frames(timeout=0.03):
            nonlocal acked_count
            frames = self._read_available(timeout=timeout)
            for _, text in frames:
                for resp_line in text.split("\n"):
                    r = resp_line.strip()
                    if not r:
                        continue
                    if r == "ok":
                        if in_flight_lens:
                            in_flight_lens.pop(0)
                        acked_count += 1
                    elif r.startswith("error:"):
                        if in_flight_lens:
                            in_flight_lens.pop(0)
                        acked_count += 1
                        print(f"\n[!] Machine reported {r} near line {acked_count}")
                    elif r.startswith("<"):
                        self._parse_status(r)

        for idx, line in enumerate(exec_lines, 1):
            line_bytes_len = len(line) + 1  # +1 for '\n'
            # Wait until buffer has room for this line
            wait_start = time.time()
            while sum(in_flight_lens) + line_bytes_len > RX_BUFFER_SIZE:
                process_frames(timeout=0.02)
                if time.time() - wait_start > 30.0:
                    # Query status to see if machine is in Hold (e.g. M0)
                    self.send_realtime("?")
                    process_frames(timeout=0.1)
                    if "Hold" in self.last_status.get("state", ""):
                        print("\n[!] Machine is in HOLD state (M0 pause). Send --resume (~) or press Enter to resume...")
                        try:
                            input()
                            self.send_realtime("~")
                        except EOFError:
                            raise RuntimeError("Machine paused in Hold state (M0) in non-interactive session.")
                    wait_start = time.time()

            self.send_line(line)
            in_flight_lens.append(line_bytes_len)
            sent_count += 1
            process_frames(timeout=0.005)

            if on_progress:
                on_progress(idx, total_lines, line)
            elif idx % 25 == 0 or idx == total_lines:
                elapsed = time.time() - start_time
                pct = (idx / total_lines) * 100
                rate = idx / elapsed if elapsed > 0 else 0
                eta = (total_lines - idx) / rate if rate > 0 else 0
                sys.stdout.write(
                    f"\r    [{idx}/{total_lines}] {pct:5.1f}% | {rate:5.1f} lines/s | ETA: {eta:4.1f}s | State: {self.last_status.get('state', 'Run')}  "
                )
                sys.stdout.flush()

        # Drain remaining acknowledgements
        drain_start = time.time()
        while in_flight_lens and (time.time() - drain_start < 15.0):
            process_frames(timeout=0.05)

        elapsed = time.time() - start_time
        print(f"\n[✓] Stream finished in {elapsed:.1f}s ({acked_count}/{total_lines} acknowledged).")


def main():
    parser = argparse.ArgumentParser(description="FluidNC CNC WebSocket Client & G-code Streamer")
    parser.add_argument("--ip", default=DEFAULT_IP, help=f"FluidNC IP address (default: {DEFAULT_IP})")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="FluidNC port (default: 80)")
    parser.add_argument("--status", action="store_true", help="Query machine status, coordinates, and SD card state")
    parser.add_argument("--unlock", action="store_true", help="Send $X to unlock machine alarm")
    parser.add_argument("--home", action="store_true", help="Send $H to run homing cycle")
    parser.add_argument("--zero-work", action="store_true", help="Send G10 L20 P1 X0 Y0 Z0 (zero work coordinates)")
    parser.add_argument("--hold", action="store_true", help="Send ! (Feed Hold)")
    parser.add_argument("--resume", action="store_true", help="Send ~ (Cycle Resume)")
    parser.add_argument("--reset", action="store_true", help="Send Ctrl+X (Soft Reset)")
    parser.add_argument("--stream", type=str, help="Stream G-code file directly over WebSocket")
    parser.add_argument("--upload", type=str, help="Upload G-code file to FluidNC SD card (/upload)")
    parser.add_argument("--run", action="store_true", help="With --upload: immediately run the uploaded file ($SD/Run)")
    parser.add_argument("--cmd", type=str, help="Send arbitrary G-code / $-command line")

    args = parser.parse_args()

    client = FluidNCClient(host=args.ip, port=args.port)

    # If no flags specified, default to status check
    if not any([args.status, args.unlock, args.home, args.zero_work, args.hold, args.resume, args.reset, args.stream, args.upload, args.cmd]):
        args.status = True

    try:
        print(f"[*] Connecting to FluidNC at ws://{args.ip}:{args.port}...")
        client.connect()
        print("[✓] Connected to FluidNC WebSocket.")
    except Exception as e:
        print(f"[✗] Error connecting: {e}")
        print("\n[!] Diagnostic Advice:")
        print("    1. Verify machine is powered and connected to Wi-Fi.")
        print("    2. If FigUI browser tab is open, close other tabs/devices (FluidNC allows 1 active WebSocket).")
        print("    3. Ping test: ping -c 1", args.ip)
        return 1

    try:
        if args.unlock:
            print("[*] Sending $X (Unlock Alarm)...")
            client.send_line("$X")
            time.sleep(0.2)
            st = client.query_status()
            print("[✓] Machine status after unlock:", st.get("state", "Unknown"))

        if args.home:
            print("[*] Sending $H (Home Machine)...")
            client.send_line("$H")
            time.sleep(0.5)

        if args.zero_work:
            print("[*] Setting Work Coordinates to (0,0,0) (G10 L20 P1 X0 Y0 Z0)...")
            client.send_line("G10 L20 P1 X0 Y0 Z0")
            time.sleep(0.2)

        if args.hold:
            print("[*] Sending Feed Hold (!)...")
            client.send_realtime("!")

        if args.resume:
            print("[*] Sending Cycle Resume (~)...")
            client.send_realtime("~")

        if args.reset:
            print("[*] Sending Soft Reset (Ctrl+X)...")
            client.send_realtime("\x18")

        if args.cmd:
            print(f"[*] Sending command: '{args.cmd}'...")
            client.send_line(args.cmd)
            time.sleep(0.2)
            frames = client._read_available(timeout=0.5)
            for _, text in frames:
                print("    FluidNC:", text.strip())

        if args.upload:
            client.upload_file(args.upload, target="sd", run_after=args.run)

        if args.stream:
            client.stream_gcode(args.stream)

        if args.status:
            st = client.query_status()
            sd_ok, sd_msg = client.check_sd_status()
            print("\n" + "=" * 50)
            print("         FLUIDNC CNC CONTROLLER STATUS         ")
            print("=" * 50)
            print(f"  Target IP:        {args.ip}:{args.port}")
            print(f"  State:            {st.get('state', 'Unknown')}")
            print(f"  Machine Pos MPos: {st.get('MPos', 'Unknown')}")
            print(f"  Work Offset WCO:  {st.get('WCO', '0.000,0.000,0.000')}")
            print(f"  Feed / Spindle:   {st.get('FS', '0,0')}")
            print(f"  SD Card Storage:  {'[OK] ' if sd_ok else '[MISSING] '} {sd_msg}")
            print("=" * 50)
            if not sd_ok:
                print("\n[!] IMPORTANT SD CARD NOTE:")
                print("    No micro-SD card is mounted on the MKS-DLC32 board.")
                print("    - WebUI drag-and-drop file upload to /sd/ requires a FAT32 micro-SD card.")
                print("    - However, you can STREAM any G-code file directly using:")
                print(f"        python3 fluidnc_sender.py --stream <file.gcode>")
                print("      without needing any SD card at all!")

    finally:
        client.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
