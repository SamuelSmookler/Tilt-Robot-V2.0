import json
import math
import time
from station.transport import RealSerial
from station.protocol import Link, CommandError


def _close_link_transport(link):
    """Close a Link's underlying serial object, if it owns one.

    The current Transport interface predates explicit resource cleanup, so
    RealSerial exposes pyserial through ``_ser``.  Keep the compatibility
    fallback here until Transport grows a public close() method.
    """
    transport = getattr(link, "_transport", None)
    close_transport = getattr(transport, "close", None)
    if callable(close_transport):
        close_transport()
        return

    serial_port = getattr(transport, "_ser", None)
    close_serial = getattr(serial_port, "close", None)
    if callable(close_serial):
        close_serial()


class RealMotion:
    """Host-side client for the two Zaber rotary stages."""


    def __init__(self, port, baud=115200, timeout_s=45,
                 startup_timeout_s=12.0):
        self._link = None
        self._closed = True
        self._timeout = timeout_s
        self._port = port
        try:
            self._link = Link(RealSerial(port, baud, timeout=1.0))
            self._closed = False
            self._wait_until_ready(startup_timeout_s)
        except BaseException:
            # If __init__ fails, the assignment in gui.py never completes, so
            # its finally block has no RealMotion object to close.  Release the
            # COM handle here or the next RUN sees Windows error 5/access denied.
            try:
                self.close()
            except Exception:
                pass
            raise

    def _wait_until_ready(self, timeout_s):
        """Wait through the Arduino reset and its blocking X-MCC probes.

        Opening the USB serial port can reset the Arduino. Its setup routine
        then probes both X-MCC axes, limits, and reference flags; those queries
        can outlast one serial read timeout. Commands lost to the bootloader or
        answered late are harmless because every retry uses a new protocol tag.
        """
        timeout_s = float(timeout_s)
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError("startup_timeout_s must be finite and > 0")

        deadline = time.monotonic() + timeout_s
        last_error = None
        attempts = 0
        while time.monotonic() < deadline:
            attempts += 1
            try:
                reply = self._link.command("PING")
                if "PONG" in reply.split():
                    return
                last_error = RuntimeError(
                    f"unexpected PING reply {reply!r}"
                )
            except Exception as exc:
                last_error = exc

            remaining = deadline - time.monotonic()
            if remaining > 0:
                time.sleep(min(0.25, remaining))

        detail = f": {last_error}" if last_error is not None else ""
        raise RuntimeError(
            f"motion controller on {self._port} did not become ready within "
            f"{timeout_s:g} s after {attempts} attempts{detail}; verify power, "
            "the stage_port setting, and that motion_arduino.ino is running"
        ) from last_error

    def ping(self):
        return self._link.command("PING")

    def status(self):
        reply = self._link.command("STATUS")
        _, body = reply.split(' ',1)
        toks = body.split()
        result = {"state": toks[0]}
        for kv in toks[1:]:
            key, value = kv.split("=", 1)
            try:
                result[key] = float(value)
            except ValueError:
                result[key] = value


        return result

    def _raise_pending_motion_fault(self):
        """Surface asynchronous Arduino events instead of silently hiding them."""
        events = getattr(self._link, "_events", None)
        if events is None:
            return
        while events:
            event = events.pop(0)
            if event.startswith("EVT FAULT"):
                detail = event[len("EVT FAULT"):].strip() or "unspecified"
                raise RuntimeError(f"motion controller fault: {detail}")
            if event.startswith("EVT ESTOP"):
                raise RuntimeError("motion controller E-stop is active")

    def _wait_idle(self):
        deadline = time.time() + self._timeout
        while time.time() < deadline:
            status = self.status()
            self._raise_pending_motion_fault()
            state = status["state"]
            if state == "IDLE":
                return status
            if state == "FAULT":
                raise RuntimeError(
                    "motion controller fault: "
                    f"{status.get('fault', 'unspecified')}"
                )
            if state == "ESTOP":
                raise RuntimeError("motion controller E-stop is active")
            if state not in ("MOVING", "HOMING"):
                raise RuntimeError(f"unexpected motion-controller state {state!r}")

            time.sleep(0.2)
        self.stop_now()
        raise TimeoutError("stage did not reach idle in time")
    def move(self, axis, deg):
        self._link.command(f"MOVE {axis} {deg}")
        self._wait_idle()

    def move_both(self, outer_deg, inner_deg):
        """Move both axes together, with a sequential fallback for old firmware."""
        try:
            self._link.command(f"MOVEB {outer_deg} {inner_deg}")
            self._wait_idle()
        except CommandError:
            self.move("OUTER", outer_deg)
            self.move("INNER", inner_deg)

    def start_scan(self, axis, target_deg, speed_deg_s):
        """Start a bounded, speed-limited absolute move without waiting for it."""
        axis = axis.upper()
        if axis not in ("OUTER", "INNER"):
            raise ValueError("scan axis must be OUTER or INNER")
        speed_deg_s = float(speed_deg_s)
        if not math.isfinite(speed_deg_s) or speed_deg_s <= 0:
            raise ValueError("scan speed must be finite and > 0")

        status = self.status()
        firmware = status.get("fw")
        if not isinstance(firmware, float) or firmware < 4.0:
            reported = "missing" if firmware is None else str(firmware)
            raise RuntimeError(
                "005 recovery scanning requires motion_arduino firmware fw=4 "
                f"or newer; controller reports fw={reported}"
            )
        try:
            self._link.command(f"SCAN {axis} {target_deg} {speed_deg_s}")
        except CommandError as exc:
            raise RuntimeError(
                "motion controller rejected the bounded 005 recovery scan; "
                "install the supplied motion_arduino.ino (fw=4)"
            ) from exc

    def home(self, axis, direction=None):
        """Find the RDQ optical index, allowing the device a full revolution.

        ``direction`` is retained only so the existing bench GUI remains
        compatible. Firmware 5 uses the stage's complete home operation.
        """
        axis = axis.upper()
        if axis not in ("OUTER", "INNER"):
            raise ValueError("home axis must be OUTER or INNER")
        self._link.command(f"HOME {axis}")
        status = self._wait_idle()
        if status.get(f"{axis.lower()}_homed") != 1.0:
            raise RuntimeError(
                f"{axis} home sweep ended without establishing a reference "
                f"(fault={status.get('fault', 'unspecified')})"
            )


    def stop(self, wait=True):
        reply = self._link.command("STOP")
        if wait:
            self._wait_idle()
        return reply
    def stop_now(self):
        """IMMEDIATE/KILL STOP. Does not wait for reply"""
        self._link._transport._ser.write(b"#0 STOP\n")
        self._link._transport._ser.flush()




    def park_at(self, outer_deg, inner_deg):
        """Return to a previously validated electrolytic-sensor null."""
        if self.status().get("homed") != 1.0:
            return False
        self.move_both(float(outer_deg), float(inner_deg))
        return True


    def close(self):
        """Release the motion-controller COM port; safe to call repeatedly."""
        if self._closed:
            return
        self._closed = True
        _close_link_transport(self._link)



class RealDUT:


    def __init__(self, port, slot=0, baud=115200):
        self._link = Link(RealSerial(port, baud, timeout=6.0))
        self._closed = False
        self._slot = slot
        self._last_sequence = None
        try:
            reply = self._link.command('RECONFIG')
            cfg = json.loads(reply.split(" ", 1)[1]).get("configured")
            if cfg ==0:
                raise RuntimeError("Teensy sees 0 DUTs configured - check power/wiring")
            else:
                deadline = time.time() + 4.0
                while time.time() < deadline:
                    valid = self.read_frame().get("units_valid")
                    if valid is None or valid[self._slot]:
                        break
                    time.sleep(0.2)

        except CommandError:
            pass


    def ping(self):
        return self._link.command("PING")

    def read_frame(self):
        reply = self._link.command("READ")
        _, body = reply.split(" ", 1)

        return json.loads(body)


        # Mistakenly wrote for the teensy
        # for pair in body.split(" "):
        #     key, value = pair.split('=')

        #     result.append(float(value))
        # return result


    def read_sample(self, timeout_s=1.0):
        """Return one fresh TP3 sample plus native pitch/roll and timing.

        Current Teensy firmware exposes a monotonically changing ``units_seq``
        value for every TP3.  Waiting for that value to change prevents a fast
        host loop from counting the same cached sample more than once.  Older
        firmware without ``units_seq`` remains supported, but cannot provide
        that freshness guarantee.

        ``units_pr`` is ordered [pitch, roll] by the Teensy firmware.  It is
        optional here so an older hub can still complete acquisition; the
        final verification report will explicitly fail missing angle metrics.
        """
        deadline = time.monotonic() + timeout_s
        while True:
            frame = self.read_frame()
            vu = frame.get("units_valid")
            if vu is not None and not vu[self._slot]:
                raise RuntimeError(f"error at: {self._slot} has no fresh sample "
                                   f"(units_valid=0) -- link or power dropout, "
                                   "try RECONFIG")

            try:
                values = [float(v) for v in frame["units"][self._slot]]
            except (KeyError, IndexError, TypeError, ValueError) as exc:
                raise RuntimeError(f"invalid TP3 sample for slot {self._slot}") from exc
            if len(values) != 3 or not all(math.isfinite(v) for v in values):
                raise RuntimeError(f"invalid TP3 vector for slot {self._slot}: {values!r}")

            pitch_roll_deg = None
            try:
                pitch_roll_deg = [
                    float(v) for v in frame["units_pr"][self._slot]
                ]
                if (len(pitch_roll_deg) != 2
                        or not all(math.isfinite(v) for v in pitch_roll_deg)):
                    pitch_roll_deg = None
            except (KeyError, IndexError, TypeError, ValueError):
                pitch_roll_deg = None

            sequences = frame.get("units_seq")
            sequence = (sequences[self._slot]
                        if sequences is not None and len(sequences) > self._slot
                        else None)
            if sequence is None or sequence != self._last_sequence:
                self._last_sequence = sequence
                sample_us = frame.get("units_sample_us")
                age_ms = frame.get("units_age_ms")
                return {
                    "values": values,
                    "pitch_roll_deg": pitch_roll_deg,
                    "sequence": sequence,
                    "sample_us": (sample_us[self._slot]
                                  if sample_us is not None and len(sample_us) > self._slot
                                  else None),
                    "age_ms": (age_ms[self._slot]
                               if age_ms is not None and len(age_ms) > self._slot
                               else None),
                    "timestamp_s": time.time(),
                }

            if time.monotonic() >= deadline:
                raise RuntimeError(f"TP3 slot {self._slot} did not produce a new "
                                   f"sample within {timeout_s:g} s")
            time.sleep(0.005)

    def read(self):
        """Compatibility wrapper returning only the fresh X/Y/Z vector."""
        return self.read_sample()["values"]


    def close(self):
        """Release the DUT-hub COM port; safe to call repeatedly."""
        if self._closed:
            return
        self._closed = True
        _close_link_transport(self._link)
