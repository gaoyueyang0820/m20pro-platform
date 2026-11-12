"""World-frame LiDAR tracks for short-lived planning occupancy."""
import math
from collections import deque
from dataclasses import dataclass, field


_NEIGHBOURS = tuple((x, y) for x in (-1, 0, 1) for y in (-1, 0, 1) if x or y)


@dataclass
class ObstacleObservation:
    current: set
    confirmed: set
    temporary: set
    cleared: set
    moving_tracks: int = 0


@dataclass
class _Track:
    samples: deque = field(default_factory=lambda: deque(maxlen=5))
    pending: set = field(default_factory=set)
    current: set = field(default_factory=set)
    moving: bool = False

    def motion(self):
        if len(self.samples) < 3:
            return False, False
        start, middle, end = self.samples[0], self.samples[len(self.samples)//2], self.samples[-1]
        span = end[0] - start[0]
        if span < .36:
            return False, False
        first = (middle[1]-start[1], middle[2]-start[2])
        second = (end[1]-middle[1], end[2]-middle[2])
        a, b = math.hypot(*first), math.hypot(*second)
        moving = (math.dist(start[1:], end[1:]) >= .32
                  and .25*(middle[0]-start[0]) <= a <= 2.8*(middle[0]-start[0])
                  and .25*(end[0]-middle[0]) <= b <= 2.8*(end[0]-middle[0])
                  and first[0]*second[0]+first[1]*second[1] >= .8*a*b)
        stationary = all(math.dist(sample[1:], end[1:]) < .16 for sample in self.samples)
        return moving, stationary


class MovingObstacles:
    SAMPLE_PERIOD = .20
    TRACK_TIMEOUT = .65
    MAX_TRACKS = 24

    def __init__(self, resolution=.2):
        self.resolution = resolution
        self.reset()

    def reset(self):
        self.tracks = []
        self.sample_stamp = None
        self.last_stamp = None
        self.pose = None
        self.context = None

    def _clusters(self, cells, scan, project, world_cells):
        remaining = set(cells)
        if len(remaining) > 4096:
            return [], set()
        boxes = scan.get('obstacle_bounds')
        bounds = dict(zip(cells, boxes)) if isinstance(boxes, list) and len(boxes) == len(cells) else {}
        groups, structures = [], set()
        while remaining:
            seed = remaining.pop()
            component = [seed]
            for x, y in component:
                for dx, dy in _NEIGHBOURS:
                    neighbour = (x+dx, y+dy)
                    if neighbour in remaining:
                        remaining.remove(neighbour)
                        component.append(neighbour)
            xs, ys = zip(*component)
            footprint = {world_cells[p] for p in component}
            if max(max(xs)-min(xs), max(ys)-min(ys))*self.resolution > .8:
                structures.update(footprint)
                continue
            centers = []
            for cell in component:
                box = bounds.get(cell)
                if isinstance(box, (list, tuple)) and len(box) == 4 and all(
                    isinstance(value, (int, float)) and math.isfinite(value) for value in box
                ):
                    centers.append(((box[0]+box[2])*.5, (box[1]+box[3])*.5))
                else:
                    centers.append(tuple(value*self.resolution for value in cell))
            x = sum(p[0] for p in centers)/len(centers)
            y = sum(p[1] for p in centers)/len(centers)
            distance = math.hypot(x, y)
            if distance <= 4:
                groups.append((distance, project(x, y), footprint))
        groups.sort(key=lambda group: (group[0], group[1]))
        return groups[:self.MAX_TRACKS], structures

    def observe(self, scan, *, cleared=None, epoch=None):
        """Only repeated stationary or extended occupancy enters long-lived memory.

        The current scan always blocks, including compact candidates and confirmed
        movers. Movement removes only a track's provisional history. Earlier
        committed occupancy still requires independently measured floor to clear.
        """
        x, y, yaw = scan['pose']
        c, s = math.cos(yaw), math.sin(yaw)
        def project(px, py):
            return x+c*px-s*py, y+s*px+c*py
        def world(cell):
            wx, wy = project(cell[0]*self.resolution, cell[1]*self.resolution)
            return round(wx/self.resolution), round(wy/self.resolution)
        cells = [tuple(p) for p in scan['obstacles']]
        world_cells = {p: world(p) for p in cells}
        current = set(world_cells.values())
        stamp = scan.get('stamp')
        valid_stamp = type(stamp) in (int, float) and math.isfinite(stamp)
        context = (epoch, scan.get('frame_id'), scan.get('pose_frame_id'), scan.get('pose_source'))
        if self.last_stamp is not None and (not valid_stamp or stamp < self.last_stamp or context != self.context):
            self.reset()
        if self.pose is not None and stamp > self.last_stamp:
            dt = min(.65, stamp-self.last_stamp)
            delta_yaw = yaw-self.pose[2]
            if math.dist((x,y),self.pose[:2]) > .30+1.8*dt or abs(math.atan2(math.sin(delta_yaw),math.cos(delta_yaw))) > .25+2.8*dt:
                self.reset()
        sample_due = valid_stamp and (self.sample_stamp is None or stamp-self.sample_stamp >= self.SAMPLE_PERIOD-1e-6)
        if cleared is None:
            cleared = {world(p) for p in scan['floor']} if sample_due or not valid_stamp else set()
        cleared = cleared-current
        if not valid_stamp:
            return ObstacleObservation(current, set(), set(), cleared)
        if stamp != self.last_stamp:
            self.last_stamp, self.pose, self.context = stamp, (x,y,yaw), context
        for track in self.tracks:
            track.pending.difference_update(cleared)
            track.current.difference_update(cleared)
        confirmed = set()
        if sample_due:
            self.sample_stamp = stamp
            groups, confirmed = self._clusters(cells, scan, project, world_cells)
            tracks = [track for track in self.tracks if stamp-track.samples[-1][0] <= self.TRACK_TIMEOUT]
            matches = []
            for i, (_, center, _) in enumerate(groups):
                for j, track in enumerate(tracks):
                    when, tx, ty = track.samples[-1]
                    distance = math.dist(center, (tx, ty))
                    if distance <= .18 + 2.8*(stamp-when):
                        matches.append((distance, i, j))
            matched, assigned = set(), {}
            for _, i, j in sorted(matches):
                if i not in assigned and j not in matched:
                    matched.add(j)
                    assigned[i] = tracks[j]
            for i, (_, center, footprint) in enumerate(groups):
                track = assigned.get(i)
                if track is None:
                    track = _Track()
                    tracks.append(track)
                track.samples.append((stamp, *center))
                track.current = footprint
                moving, stationary = track.motion()
                track.moving |= moving
                if track.moving:
                    track.pending.clear()
                elif stationary:
                    confirmed.update(footprint)
                    track.pending.clear()
                else:
                    track.pending.update(footprint)
            # New observations take priority over tracks currently occluded.
            tracks.sort(key=lambda track: track.samples[-1][0], reverse=True)
            self.tracks = tracks[:self.MAX_TRACKS]
        temporary = set()
        for track in self.tracks:
            temporary.update(track.pending)
            if track.moving:
                temporary.update(track.current)
        return ObstacleObservation(current, confirmed, temporary, cleared,
                                   sum(track.moving for track in self.tracks))
