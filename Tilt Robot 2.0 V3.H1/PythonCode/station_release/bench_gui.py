import tkinter as tk
from tkinter import ttk
import threading, queue, time
from station.hardware import RealMotion, RealDUT

STAGE_PORT, DUT_PORT, DUT_SLOT = "COM7", "COM6", 0
POLL_S = 0.3

root = tk.Tk()
root.title("Bench- TESTING")
root.geometry("560x400")

vals = queue.Queue()
state = {"outer": "-", "inner": "-", "dut": "-", "conn": "connecting..."}

cmds = queue.Queue()
hw = {"motion": None}


def row(parent, label):
    """One label: value line. Returns the value widget so we can update it """
    f = ttk.Frame(parent); f.pack(fill= "x", pady=2)
    ttk.Label(f, text=label, width=22).pack(side="left")
    v = ttk.Label(f, text = "-", font=("Consolas", 11))
    v.pack(side="left")
    return v 

stage_box = ttk.LabelFrame(root, text="Rotary stages", padding=10)
stage_box.pack(fill="x", padx=10, pady=8)
outer_lbl = row(stage_box, "Roll / OUTER (deg)")
inner_lbl = row(stage_box, "Pitch / INNER (deg)")

dut_box = ttk.LabelFrame(root, text="DUT (slot 0)", padding=10)
dut_box.pack(fill="x", padx=10, pady=8)
dut_lbl = row(dut_box, "accel [x y z] (g)")

conn_lbl = ttk.Label(root, text="connecting...", padding=(10,4))
conn_lbl.pack(fill="x")

def poller():
    """BACKGROUND thread. Never touches widget - only vals.put()."""
    try:
        motion = RealMotion(STAGE_PORT)
        dut = RealDUT(DUT_PORT, slot=DUT_SLOT)
        hw["motion"] = motion
        vals.put(("conn", "connected"))
    except Exception as e:
        vals.put(("conn", f"connect failed: {e}"))
        return
    while True:
        try: 
            while not cmds.empty(): 
                kind, axis, arg = cmds.get()
                vals.put(("conn", f"{kind} {axis} {arg} ..."))
                if kind == "move":
                    motion.move(axis, arg)
                elif kind == "home":
                    motion.home(axis, arg)
                vals.put(("conn", "idle"))
            
        
            
            
            
            
            
            
            
            st = motion.status()
            vals.put(("outer", f"{st.get('outer', float('nan')):+9.4f}"))
            vals.put(("inner", f"{st.get('inner', float('nan')):+9.4f}"))
            x,y,z = dut.read()
            vals.put(("dut", f"[{x:+.4f} {y:+.4f} {z:+.4f}] "))
        
        
        except Exception as e:
            vals.put(("conn", f"read error: {e}"))
        time.sleep(POLL_S)
        
        
ctl = ttk.LabelFrame(root, text="Control", padding=10)
ctl.pack(fill="x", padx=10, pady = 8)

def axis_row(parent, axis):
    f = ttk.Frame(parent); f.pack(fill="x", pady=3)
    ttk.Label(f, text=axis, width=8).pack(side="left")
    entry = ttk.Entry(f, width=10); entry.insert(0,"0.0"); entry.pack(side="left", padx=4)

    def do_move():
        try:
            deg = float(entry.get())
        except ValueError:
            vals.put(("conn", "bad angle")); return
        cmds.put(("move", axis, deg))
    ttk.Button(f, text="Move", command=do_move).pack(side="left", padx=3)
    ttk.Button(f, text="Home Neg", command=lambda: cmds.put(("home", axis, "NEG"))).pack(side="left", padx=3)

axis_row(ctl, "OUTER")
axis_row(ctl, "INNER")


def on_stop():
    m = hw["motion"]
    if m:
        try:
            m.stop_now()
            vals.put(("conn", "STOP sent"))
        except Exception as e:
            vals.put(("conn", f"STOP FAILED: {e}"))
    else:
        vals.put(("conn", "STOP ignored -- not connected"))
        
stop = ttk.Button(ctl, text="STOP", command=on_stop)
stop.pack(side="left", padx=12)


        
def pump():
    while not vals.empty():
        key , text  = vals.get()
        if   key   == "outer": outer_lbl.config(text=text)
        elif key   == "inner": inner_lbl.config(text=text)
        elif key   == "dut": dut_lbl.config(text=text)
        elif key   == "conn": conn_lbl.config(text=text)
        
    root.after(100, pump)
    
    
    
threading.Thread(target=poller, daemon=True).start()
root.after(100, pump)
    
root.mainloop()