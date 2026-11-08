#!/usr/bin/env python3

import socket
import threading
import time

import serial


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

NANO_PORT = "/dev/ttyUSB1"
NANO_BAUD = 115200

TCP_HOST = "127.0.0.1"
TCP_PORT = 4030

# SkyWatcher motor-controller identification.
# This produces: =040200\r
MOTOR_VERSION = 0x000204

AZ_MICROSTEPS_PER_REV = 0x162B97
ALT_MICROSTEPS_PER_REV = 0x162B97

AZ_CLOCK_FREQUENCY = 0x00C350
ALT_CLOCK_FREQUENCY = 0x00C350

AZ_HIGH_SPEED_RATIO = 0x000100
ALT_HIGH_SPEED_RATIO = 0x000100

AZ_WORM_REVOLUTION = 0
ALT_WORM_REVOLUTION = 0

AZ_ENCODER_SCALE = 1.0
ALT_ENCODER_SCALE = 1.0

AZ_INITIAL_POSITION = 0
ALT_INITIAL_POSITION = 0


def log(message):
    print(message, flush=True)


def hex24(value):
    value &= 0xFFFFFF
    return (
        f"{value & 0xFF:02X}"
        f"{(value >> 8) & 0xFF:02X}"
        f"{(value >> 16) & 0xFF:02X}"
    )


def reply(payload=""):
    return ("=" + payload + "\r").encode("ascii")


class SkyWatcherBridge:
    def __init__(self):
        self.nano = serial.Serial(
            NANO_PORT,
            NANO_BAUD,
            timeout=1,
            write_timeout=1,
        )

        self.positions = {
            1: AZ_INITIAL_POSITION,
            2: ALT_INITIAL_POSITION,
        }

        self.lock = threading.Lock()

        log(f"Nano connected: {NANO_PORT}")
        log(f"TCP listening on {TCP_HOST}:{TCP_PORT}")

    def nano_command(self, command):
        line = (command + "\n").encode("ascii")

        with self.lock:
            self.nano.reset_input_buffer()
            self.nano.write(line)
            self.nano.flush()

            deadline = time.monotonic() + 2.0

            while time.monotonic() < deadline:
                line = self.nano.readline()

                if not line:
                    continue

                text = line.decode("ascii", errors="replace").strip()
                log(f"Nano -> {text}")

                if text.startswith("ERROR:"):
                    raise RuntimeError(text)

                if text.startswith("RX:"):
                    return text

        raise TimeoutError(f"Nano timeout: {command}")

    @staticmethod
    def parse_rx(text):
        values = []

        for token in text.removeprefix("RX:").split():
            if token.startswith("BIT="):
                continue

            try:
                values.append(int(token, 16))
            except ValueError:
                pass

        return values

    def read_position(self, axis):
        response_text = self.nano_command(f"{axis} 08")
        values = self.parse_rx(response_text)

        if len(values) < 2:
            raise RuntimeError(f"Invalid Nano response: {response_text}")

        raw = (values[0] << 8) | values[1]

        if raw & 0x8000:
            raw -= 0x10000

        scale = AZ_ENCODER_SCALE if axis == 1 else ALT_ENCODER_SCALE
        self.positions[axis] += int(raw * scale)

        return self.positions[axis]

    def process_command(self, command):
        command = command.strip("\r\n")

        if not command:
            return b""

        # INDI motor-type probe. Do not answer it.
        if command == ":":
            log("INDI motor probe ignored")
            return b""

        # Handle accidental combined probe and command.
        if command.startswith("::"):
            command = command[1:]

        if not command.startswith(":"):
            log(f"Ignoring invalid command: {command!r}")
            return b""

        log(f"TCP -> {command!r}")

        body = command[1:]

        if len(body) < 2:
            return b"!\r"

        operation = body[0]
        axis_text = body[1]

        if axis_text not in ("1", "2"):
            return b"!2\r"

        axis = int(axis_text)

        if operation == "e" and axis == 1:
            return reply(hex24(MOTOR_VERSION))

        if operation == "a":
            value = (
                AZ_MICROSTEPS_PER_REV
                if axis == 1
                else ALT_MICROSTEPS_PER_REV
            )
            return reply(hex24(value))

        if operation == "b":
            value = (
                AZ_CLOCK_FREQUENCY
                if axis == 1
                else ALT_CLOCK_FREQUENCY
            )
            return reply(hex24(value))

        if operation == "g":
            value = (
                AZ_HIGH_SPEED_RATIO
                if axis == 1
                else ALT_HIGH_SPEED_RATIO
            )
            return reply(hex24(value))

        if operation == "s":
            value = (
                AZ_WORM_REVOLUTION
                if axis == 1
                else ALT_WORM_REVOLUTION
            )
            return reply(hex24(value))

        if operation == "j":
            return reply(hex24(self.read_position(axis)))

        if operation == "f":
            # Stopped, inactive.
            return reply("000000")

        if operation in ("F", "J", "K", "L", "M", "U", "O"):
            return reply()

        if operation in ("G", "H", "I"):
            return reply()

        log(f"Unsupported command: {command!r}")
        return b"!3\r"

    def client_thread(self, connection, address):
        log(f"Client connected: {address}")
        buffer = b""

        try:
            connection.settimeout(1.0)

            while True:
                try:
                    data = connection.recv(4096)
                except socket.timeout:
                    continue

                if not data:
                    break

                log(f"TCP RX raw: {data!r}")
                buffer += data

                while b"\r" in buffer:
                    frame, buffer = buffer.split(b"\r", 1)

                    if not frame:
                        continue

                    try:
                        result = self.process_command(
                            frame.decode("ascii", errors="ignore")
                        )
                    except Exception as exc:
                        log(f"Command error: {exc}")
                        result = b"!2\r"

                    if result:
                        connection.sendall(result)
                        log(f"TCP <- {result!r}")

        except OSError as exc:
            log(f"Client error: {exc}")
        finally:
            connection.close()
            log(f"Client disconnected: {address}")

    def run(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((TCP_HOST, TCP_PORT))
        server.listen(1)

        try:
            while True:
                connection, address = server.accept()

                thread = threading.Thread(
                    target=self.client_thread,
                    args=(connection, address),
                    daemon=True,
                )
                thread.start()

        finally:
            server.close()
            self.nano.close()


def main():
    bridge = SkyWatcherBridge()

    try:
        bridge.run()
    except KeyboardInterrupt:
        log("Stopping bridge.")
    finally:
        bridge.nano.close()


if __name__ == "__main__":
    main()