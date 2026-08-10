import numpy as np
import itertools

#review this as im not entirely ure how it comes into play
PHI = (1 + np.sqrt(5))/2

OUTER_WINDOW = (0.0, 360.0)

def _dev(deg, window=OUTER_WINDOW):
    """Protocol's angle -> device coordinate, """
    lo, hi = window
    for cand in (deg, deg + 360.0, deg - 360.0):
        if lo - 1e-6 <= cand <= hi + 1e-6:
            return cand
    return lo + ((deg - lo) % 360.0)
        


        
def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0



def travel_cost(a,b, simultaneous=True, window=OUTER_WINDOW, outer_offset=0.0):
    ao = _dev(wrap180(a[0] + outer_offset), window)
    bo = _dev(wrap180(b[0] + outer_offset), window)
    do = abs(bo - ao)
    di = abs(b[1] - a[1])
    return max(do, di) if simultaneous else do + di

def tour_length(poses, start=(0.0, 0.0), simultaneous=True,
                window=OUTER_WINDOW, outer_offset=0.0):
            total, cur = 0.0, start
            for p in poses:
                total += travel_cost(cur, p, simultaneous, window, outer_offset)
                cur = p 
            return total               
                
                

def symmetric_directions():
    """The 26-pose symmetric core: 6 faces + 8 cube corners + 12 edge centers"""
    faces = [(1,0,0), (-1,0,0), (0,1,0), (0,-1,0), (0,0,1), (0, 0, -1)]
    #cool line of code that makes sure every corner is hit, overcomplicated though. 
    corners = [(x,y,z) for x,y,z in itertools.product((1,-1), repeat=3)]
    edges = []
    for a,b in itertools.product((1,-1), repeat=2):
        edges += [(a,b,0), (a,0,b), (0,a,b)]
    return {"face": faces, "corner": corners, "edge": edges}


def order_poses(poses, start=(0.0, 0.0), simultaneous=True, 
                window=OUTER_WINDOW, outer_offset=0.0):
    remaining = list(poses)
    if len(remaining) < 3:
        return remaining
    
    
    tour, cur = [], start
    while remaining:
        nxt = min(remaining, key=lambda p: travel_cost(cur, p, simultaneous, window, outer_offset))
        
        remaining.remove(nxt)
        tour.append(nxt)
        cur = nxt
        
    def seg_cost(prev_pt, seg, next_pt):
        c = travel_cost(prev_pt, seg[0], simultaneous, window, outer_offset)
        for a, b in zip(seg, seg[:1]):
            c += travel_cost(a, b, simultaneous, window, outer_offset)
        if next_pt is not None:
            c += travel_cost(seg[-1], next_pt, simultaneous, window, outer_offset)
        return c
        
    improved = True
    while improved:
        improved = False
        for i in range(len(tour) -1):
            for j in range(i + 1, len(tour)):
                prev_pt = tour[i - 1] if i > 0 else start
                next_pt = tour[j + 1] if j + 1 < len(tour) else None
                seg = tour[i:j + 1]
                if (seg_cost(prev_pt, seg[::-1], next_pt)
                    < seg_cost(prev_pt, seg, next_pt) - 1e-9):
                    tour[i:j + 1] = seg[::-1]
                    improved = True
    return tour
    
    
        
def angles_for_direction(g, inner_limit=120.0):
    """table angles using gravity direction. """
    g = np.asarray(g, float); g = g / np.linalg.norm(g)
    x , y, z = g
    o1 = np.degrees(np.arcsin(y))
    i1 = np.degrees(np.arctan2(-x, z))
    o2, i2 = wrap180(180.0 - o1), wrap180(i1 - 180.0)
    candidates = [(o1, i1), (o2,i2)]
    ok = [(o,i) for (o, i) in candidates if abs(i) <= inner_limit]
    
    
    
    if not ok:
        raise ValueError(f"direction {g} unreachable with inner limit +/- {inner_limit}")
    
    
    return min(ok, key=lambda pair: abs(pair[1]))















def verification_directions():
    """12 icosahedron-vertex directions"""
    
    dirs = []
    for a,b in itertools.product((1,-1), repeat =2):
        dirs += [(0, a, b*PHI), (a, b*PHI, 0), (a*PHI, 0, b)]
    return [tuple(np.array(d) / np.linalg.norm(d)) for d in dirs]


def build_pose_set(inner_limit=120.0):
    "returns fit, verify, and dropped poses as inner and outer angle pairs"
    fams = symmetric_directions()
    dropped = []
    
    def to_angles(dirs, label):
        out = []
        for d in dirs:
            try:
                out.append(angles_for_direction(d, inner_limit))
            except ValueError:
                dropped.append((label, d))
        return out

    fit_by_family = {name: to_angles(dirs, name) for name, dirs in fams.items()}
    
    fit = []
    
    
    
    for group in itertools.zip_longest(fit_by_family['face'], fit_by_family['corner'], fit_by_family['edge']):
        fit += [p for p in group if p is not None]
        
        
    verify = to_angles(verification_directions(), "verify")
    return fit, verify, dropped

