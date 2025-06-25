import json
import copy
import math
import numpy as np
from scipy.interpolate import interp1d


class LaneletMap:
    """Lanelet 맵 데이터를 로드하고 관리하는 클래스"""
    def __init__(self, map_path):
        with open(map_path, 'r') as f:
            map_data = json.load(f)
        self.map_data = map_data
        self.lanelets = map_data['lanelets']
        self.groups = map_data['groups']
        self.precision = map_data['precision']
        self.for_viz = map_data['for_vis']
        self.base_lla = map_data['base_lla']


class TileMap:
    """타일 기반으로 맵을 분할하여 효율적인 검색을 지원하는 클래스"""
    def __init__(self, lanelets, tile_size):
        self.tiles = {}
        self.tile_size = tile_size
        self._build_tiles(lanelets)

    def _build_tiles(self, lanelets):
        for id_, data in lanelets.items():
            for n, (x, y) in enumerate(data['waypoints']):
                row = int(x // self.tile_size)
                col = int(y // self.tile_size)
                
                if (row, col) not in self.tiles:
                    self.tiles[(row, col)] = {}
                
                if id_ not in self.tiles[(row, col)]:
                    self.tiles[(row, col)][id_] = {'waypoints': [], 'idx': []}
                
                self.tiles[(row, col)][id_]['waypoints'].append((x, y))
                self.tiles[(row, col)][id_]['idx'].append(n)


class MicroLaneletGraph:
    """Lanelet을 작은 구간으로 분할하여 세밀한 그래프를 생성하는 클래스"""
    def __init__(self, lmap, cut_dist):
        self.cut_dist = cut_dist
        self.precision = lmap.precision
        self.lanelets = lmap.lanelets
        self.groups = lmap.groups
        self.graph = {}
        self.reversed_graph = {}
        self._generate_graph()

    def _generate_graph(self):
        self._cut_lanelets()
        self._build_graph()
        self._build_reversed_graph()

    def _cut_lanelets(self):
        cut_idx = int(self.cut_dist / self.precision)
        
        for group in self.groups:
            group = copy.copy(group)
            
            if self.lanelets[group[0]]['length'] > self.lanelets[group[-1]]['length']:
                group.reverse()
            
            idx_num = self.lanelets[group[0]]['idx_num']
            cut_num = (idx_num + cut_idx - 1) // cut_idx  # ceiling division
            
            for n, id_ in enumerate(group):
                self.lanelets[id_]['cut_idx'] = []
                
                if n == 0:
                    self._cut_first_lanelet(id_, cut_num, cut_idx, idx_num)
                else:
                    self._cut_following_lanelet(id_, group, n, cut_num)

    def _cut_first_lanelet(self, id_, cut_num, cut_idx, idx_num):
        for i in range(cut_num):
            start_idx = i * cut_idx
            end_idx = min(start_idx + cut_idx, idx_num)
            self.lanelets[id_]['cut_idx'].append([start_idx, end_idx])

    def _cut_following_lanelet(self, id_, group, n, cut_num):
        for i in range(cut_num):
            pre_id = group[n-1]
            pre_end_idx = self.lanelets[pre_id]['cut_idx'][i][1] - 1
            pt = self.lanelets[pre_id]['waypoints'][pre_end_idx]
            
            if i == 0:
                start_idx = 0
                end_idx = self._find_nearest_idx(self.lanelets[id_]['waypoints'], pt)
            elif i == cut_num - 1:
                start_idx = self.lanelets[id_]['cut_idx'][i-1][1]
                end_idx = self.lanelets[id_]['idx_num']
            else:
                start_idx = self.lanelets[id_]['cut_idx'][i-1][1]
                end_idx = self._find_nearest_idx(self.lanelets[id_]['waypoints'], pt)
            
            self.lanelets[id_]['cut_idx'].append([start_idx, end_idx])

    def _build_graph(self):
        for id_, data in self.lanelets.items():
            if data['group'] is None:
                self._add_single_lanelet_edges(id_, data)
            else:
                self._add_micro_lanelet_edges(id_, data)

    def _add_single_lanelet_edges(self, id_, data):
        if id_ not in self.graph:
            self.graph[id_] = {}
        
        for p_id in data['successor']:
            target_id = p_id if self.lanelets[p_id]['group'] is None else f"{p_id}_0"
            self.graph[id_][target_id] = data['length']

    def _add_micro_lanelet_edges(self, id_, data):
        last = len(data['cut_idx']) - 1
        
        for n in range(len(data['cut_idx'])):
            new_id = f"{id_}_{n}"
            if new_id not in self.graph:
                self.graph[new_id] = {}
            
            if n == last:
                self._add_final_micro_edges(new_id, data)
            else:
                self._add_intermediate_micro_edges(new_id, id_, data, n)

    def _add_final_micro_edges(self, new_id, data):
        for p_id in data['successor']:
            target_id = p_id if self.lanelets[p_id]['group'] is None else f"{p_id}_0"
            self.graph[new_id][target_id] = self.cut_dist

    def _add_intermediate_micro_edges(self, new_id, id_, data, n):
        # Next segment
        self.graph[new_id][f"{id_}_{n+1}"] = self.cut_dist
        
        s_idx, e_idx = self.lanelets[id_]['cut_idx'][n]
        mid_range = (e_idx - s_idx) // 2
        
        # Left lane change
        left_id = data['adjacentLeft']
        if left_id and self._can_change_lane(data['leftChange'], s_idx, mid_range):
            self.graph[new_id][f"{left_id}_{n+1}"] = self.cut_dist + 10.0 + n * 0.1
        
        # Right lane change
        right_id = data['adjacentRight']
        if right_id and self._can_change_lane(data['rightChange'], s_idx, mid_range):
            self.graph[new_id][f"{right_id}_{n+1}"] = self.cut_dist + 10.0 + n * 0.1

    def _can_change_lane(self, change_list, start_idx, range_size):
        return sum(change_list[start_idx:start_idx + range_size]) == range_size

    def _build_reversed_graph(self):
        for from_id, targets in self.graph.items():
            for to_id in targets:
                if to_id not in self.reversed_graph:
                    self.reversed_graph[to_id] = []
                self.reversed_graph[to_id].append(from_id)

    @staticmethod
    def _find_nearest_idx(pts, pt):
        min_dist = float('inf')
        min_idx = 0
        
        for idx, pt1 in enumerate(pts):
            dist = np.sqrt((pt[0] - pt1[0])**2 + (pt[1] - pt1[1])**2)
            if dist < min_dist:
                min_dist = dist
                min_idx = idx
        
        return min_idx


class QuadraticSplineInterpolate:
    """2차 스플라인 보간을 수행하는 클래스"""
    def __init__(self, x, y):
        self.s = self._calc_s(x, y)
        self.sx = interp1d(self.s, x, fill_value="extrapolate")
        self.sy = interp1d(self.s, y, fill_value="extrapolate")

    def _calc_s(self, x, y):
        dx = np.diff(x)
        dy = np.diff(y)
        self.ds = [math.sqrt(idx**2 + idy**2) for idx, idy in zip(dx, dy)]
        s = [0]
        s.extend(np.cumsum(self.ds))
        return s

    def calc_position(self, s):
        return self.sx(s), self.sy(s)

    def calc_yaw(self, s):
        dx = self._calc_derivative(self.sx, s)
        dy = self._calc_derivative(self.sy, s)
        return math.atan2(dy, dx)

    def calc_curvature(self, s):
        dx = self._calc_derivative(self.sx, s)
        ddx = self._calc_second_derivative(self.sx, s)
        dy = self._calc_derivative(self.sy, s)
        ddy = self._calc_second_derivative(self.sy, s)
        return (ddy * dx - ddx * dy) / ((dx**2 + dy**2)**(3/2))

    def _calc_derivative(self, sp, x, dx=1.0):
        return (sp(x + dx) - sp(x - dx)) / dx

    def _calc_second_derivative(self, sp, x, dx=2.0):
        ddp = self._calc_derivative(sp, x + dx)
        ddm = self._calc_derivative(sp, x - dx)
        return (ddp - ddm) / dx