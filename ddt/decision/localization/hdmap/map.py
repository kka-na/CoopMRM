import json
import pickle
import os
from .map_core import LaneletMap, TileMap, MicroLaneletGraph
from .map_viz import MapVisualizer


class MAP:
    """HD 맵 관리 및 처리를 위한 메인 클래스"""
    
    def __init__(self, map_name, cut_dist=15, tile_size=5):
        """
        Args:
            map_name: 맵 파일명 (확장자 제외)
            cut_dist: Micro lanelet 분할 거리
            tile_size: 타일 크기
        """
        self.map_name = map_name
        self.cut_dist = cut_dist
        self.tile_size = tile_size
        
        self.map_file = f'./localization/hdmap/maps/{map_name}.json'
        self.pickle_file = f'./localization/hdmap/pkls/{map_name}.pkl'
        
        self._load_or_create_map()

    def _load_or_create_map(self):
        """캐시된 맵 데이터를 로드하거나 새로 생성"""
        if os.path.exists(self.pickle_file):
            self._load_from_cache()
        else:
            self._create_and_cache_map()

    def _load_from_cache(self):
        """피클 파일에서 맵 데이터 로드"""
        with open(self.pickle_file, 'rb') as file:
            data = pickle.load(file)
            self.base_lla = data['base_lla']
            self.lmap = data['lmap']
            self.graph = data['graph']
            self.lanelets = data['lanelets']
            self.tiles = data['tiles']
            self.lmap_viz = data['lmap_viz']
            self.mlmap_viz = data['mlmap_viz']

    def _create_and_cache_map(self):
        """맵 데이터를 새로 생성하고 캐시에 저장"""
        if not os.path.exists(self.map_file):
            raise FileNotFoundError(f"Map file not found: {self.map_file}")
        
        # 기본 맵 데이터 로드
        with open(self.map_file, 'r') as file:
            data = json.load(file)
        self.base_lla = data['base_lla']
        
        # 맵 구조 생성
        self.lmap = LaneletMap(self.map_file)
        tmap = TileMap(self.lmap.lanelets, self.tile_size)
        mlg = MicroLaneletGraph(self.lmap, self.cut_dist)
        
        # 결과 저장
        self.graph = mlg.graph
        self.lanelets = mlg.lanelets
        self.tiles = tmap.tiles
        
        # 시각화 데이터 생성
        self.lmap_viz = MapVisualizer.create_lanelet_viz(self.lanelets, self.lmap.for_viz)
        self.mlmap_viz = MapVisualizer.create_micro_graph_viz(self.lanelets, self.graph)
        
        # 캐시에 저장
        self._save_to_cache()

    def _save_to_cache(self):
        """맵 데이터를 피클 파일로 저장"""
        os.makedirs(os.path.dirname(self.pickle_file), exist_ok=True)
        
        cache_data = {
            'base_lla': self.base_lla,
            'lmap': self.lmap,
            'graph': self.graph,
            'lanelets': self.lanelets,
            'tiles': self.tiles,
            'lmap_viz': self.lmap_viz,
            'mlmap_viz': self.mlmap_viz
        }
        
        with open(self.pickle_file, 'wb') as file:
            pickle.dump(cache_data, file)

    def get_lanelet_viz(self):
        """Lanelet 시각화 마커 배열 반환"""
        return self.lmap_viz

    def get_micro_graph_viz(self):
        """Micro lanelet 그래프 시각화 마커 배열 반환"""
        return self.mlmap_viz

    def get_tile(self, x, y):
        """주어진 좌표에 해당하는 타일 반환"""
        row = int(x // self.tile_size)
        col = int(y // self.tile_size)
        return self.tiles.get((row, col), {})

    def get_lanelet_by_id(self, lanelet_id):
        """ID로 lanelet 데이터 조회"""
        return self.lanelets.get(lanelet_id)

    def get_graph_neighbors(self, node_id):
        """그래프에서 특정 노드의 이웃 노드들 반환"""
        return self.graph.get(node_id, {})

    def get_predecessors(self, node_id):
        """특정 노드의 선행 노드들 반환"""
        return getattr(self, 'reversed_graph', {}).get(node_id, [])

    def refresh_cache(self):
        """캐시를 삭제하고 맵을 다시 생성"""
        if os.path.exists(self.pickle_file):
            os.remove(self.pickle_file)
        self._create_and_cache_map()

    def __repr__(self):
        return f"MAP(name='{self.map_name}', lanelets={len(self.lanelets)}, tiles={len(self.tiles)})"