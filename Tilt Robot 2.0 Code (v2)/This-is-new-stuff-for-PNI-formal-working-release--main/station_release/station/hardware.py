import json
import time
import os
import math
os.makedirs("runs", exist_ok=True)
from station.transport import RealSerial
from station.protocol import Link, CommandError

PARKING_ANGLE = 30

class RealMotion:
    """FOr the zaber stages"""
    
    
    def __init__(self, port, baud=115200, timeout_s=45,):
        self._link = Link(RealSerial(port, baud, timeout=1.0))
        self._timeout = timeout_s
        
    def ping(self):
        return self._link.command("PING")
    
    def status(self):
        reply = self._link.command("STATUS")
        _, body = reply.split(' ',1)
        toks = body.split()
        result = {"state": toks[0]}
        for kv in toks[1:]:
            key, value = kv.split("=")
            try: 
                result[key] = float(value)
            except ValueError:
                result[key] = value
            
            
        return result
    
    def _wait_idle(self):
        deadline = time.time() + self._timeout
        while time.time() < deadline:
            if self.status()["state"] not in ("MOVING", "HOMING"):
                return
        
            time.sleep(0.2)
        raise TimeoutError("stage did not reach idle in time")
    def move(self, axis, deg):
        self._link.command(f"MOVE {axis} {deg}")
        self._wait_idle()
    def home(self, axis, direction):
        self._link.command(f"HOME {axis} {direction}")
        self._wait_idle()
    
    
    def stop(self):
        return self._link.command("STOP")
    def stop_now(self):
        """IMMEDIATE/KILL STOP. Does not wait for reply"""
        self._link._transport._ser.write(b"#0 STOP\n")
        self._link._transport._ser.flush()
        
        
        
        
    def park(self, angle=PARKING_ANGLE):
        """Leave both axis at a positive angle before homing. """
        if self.status().get("homed") != 1.0:
            return
        
        self.move("OUTER", angle)
        self.move("INNER", angle)

        
    def close(self):
        pass
    
    
    
class RealDUT:
    
    
    def __init__(self, port, slot=0, baud=115200):
        self._link = Link(RealSerial(port, baud, timeout=6.0))
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
        """Return one fresh TP3 sample plus the hub's timing metadata.

        Current Teensy firmware exposes a monotonically changing ``units_seq``
        value for every TP3.  Waiting for that value to change prevents a fast
        host loop from counting the same cached sample more than once.  Older
        firmware without ``units_seq`` remains supported, but cannot provide
        that freshness guarantee.
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
        pass
