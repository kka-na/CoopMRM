import numpy as np
import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from .map_core import QuadraticSplineInterpolate


class MapVisualizer:
    """맵 시각화를 위한 유틸리티 클래스"""
    
    @staticmethod
    def create_lanelet_viz(lanelets, for_viz):
        """Lanelet 맵 시각화 마커 배열 생성"""
        array = MarkerArray()
        
        # Lanelet bounds 시각화
        for id_, data in lanelets.items():
            # Left bounds
            for n, (left_bound, left_type) in enumerate(zip(data['leftBound'], data['leftType'])):
                marker = MapVisualizer._create_bound_marker(
                    'leftBound', id_, n, left_bound, left_type, (1.0, 1.0, 1.0, 1.0)
                )
                array.markers.append(marker)
            
            # Right bounds
            for n, (right_bound, right_type) in enumerate(zip(data['rightBound'], data['rightType'])):
                marker = MapVisualizer._create_bound_marker(
                    'rightBound', id_, n, right_bound, right_type, (1.0, 1.0, 1.0, 1.0)
                )
                array.markers.append(marker)
        
        # 추가 시각화 요소들 (stop_line 등)
        for n, (points, type_) in enumerate(for_viz):
            color = (1.0, 1.0, 1.0, 1.0)
            line_type = 'solid' if type_ == 'stop_line' else type_
            marker = MapVisualizer._create_bound_marker(
                'for_viz', n, n, points, line_type, color
            )
            array.markers.append(marker)
        
        return array

    @staticmethod
    def create_micro_graph_viz(lanelets, graph):
        """Micro Lanelet 그래프 시각화 마커 배열 생성"""
        array = MarkerArray()
        
        for n, (node_id, edges) in enumerate(graph.items()):
            # 노드 시각화
            node_marker = MapVisualizer._create_node_marker(node_id, n, lanelets)
            array.markers.append(node_marker)
            
            # 엣지 시각화
            for m, target_node_id in enumerate(edges.keys()):
                edge_markers = MapVisualizer._create_edge_markers(
                    node_id, target_node_id, n, m, lanelets
                )
                array.markers.extend(edge_markers)
        
        return array

    @staticmethod
    def _create_node_marker(node_id, n, lanelets):
        """노드 마커 생성"""
        parts = node_id.split('_')
        
        if len(parts) == 1:
            id_ = parts[0]
            idx = lanelets[id_]['idx_num'] // 2
        else:
            id_ = parts[0]
            cut_n = int(parts[1])
            idx = sum(lanelets[id_]['cut_idx'][cut_n]) // 2
        
        pt = lanelets[id_]['waypoints'][idx]
        return MarkerFactory.create_text_marker(
            'graph_id', n, 1.5, (1.0, 1.0, 1.0, 1.0), node_id, pt
        )

    @staticmethod
    def _create_edge_markers(from_node, to_node, n, m, lanelets):
        """엣지 마커들 생성 (라인 + 화살표)"""
        from_parts = from_node.split('_')
        to_parts = to_node.split('_')
        
        # 시작점 계산
        if len(from_parts) == 1:
            from_id = from_parts[0]
            from_idx = lanelets[from_id]['idx_num'] // 2
        else:
            from_id = from_parts[0]
            cut_n = int(from_parts[1])
            from_idx = sum(lanelets[from_id]['cut_idx'][cut_n]) // 2
        
        from_pts = lanelets[from_id]['waypoints']
        
        # 끝점 계산
        if len(to_parts) == 1:
            to_id = to_parts[0]
            to_idx = lanelets[to_id]['idx_num'] // 2
            to_pts = lanelets[to_id]['waypoints']
            
            if from_id == to_id:  # 같은 lanelet 내에서의 연결
                pts = [from_pts[from_idx], to_pts[to_idx]]
            else:
                pts = from_pts[from_idx:] + to_pts[:to_idx]
        else:
            to_id = to_parts[0]
            cut_n = int(to_parts[1])
            to_idx = sum(lanelets[to_id]['cut_idx'][cut_n]) // 2
            to_pts = lanelets[to_id]['waypoints']
            
            if from_id == to_id:  # 같은 lanelet 내에서의 연결
                pts = [from_pts[from_idx], to_pts[to_idx]]
            else:
                pts = from_pts[from_idx:] + to_pts[:to_idx]
        
        # 색상 결정 (같은 lanelet 내부 연결인지 여부에 따라)
        alpha = 0.3 if from_id == to_id else 0.5
        color = (0.0, 1.0, 0.0, alpha)
        
        return MapVisualizer._create_edge_line_and_arrow(n * 100000 + m, pts, color)

    @staticmethod
    def _create_edge_line_and_arrow(marker_id, points, color):
        """엣지의 라인과 화살표 마커 생성"""
        # 2점만 있는 경우 스플라인 보간으로 부드러운 곡선 생성
        if len(points) == 2:
            wx, wy = zip(*points)
            itp = QuadraticSplineInterpolate(list(wx), list(wy))
            pts = []
            for ds in np.arange(0.0, itp.s[-1], 0.5):
                pts.append(itp.calc_position(ds))
            points = pts
        
        # 라인 마커
        line_marker = MarkerFactory.create_line_marker(
            'edge_line', marker_id, 0.2, color, points
        )
        
        # 화살표 마커
        arrow_marker = MarkerFactory.create_arrow_marker(
            'edge_arrow', marker_id, (0.3, 0.8, 1.0), color, points
        )
        
        return [line_marker, arrow_marker]

    @staticmethod
    def _create_bound_marker(ns, id_, n, points, line_type, color):
        """경계선 마커 생성"""
        if line_type == 'solid':
            marker = MarkerFactory.create_line_marker(f'{ns}_{id_}', n, 0.15, color, points)
        elif line_type == 'dotted':
            marker = MarkerFactory.create_points_marker(f'{ns}_{id_}', n, 0.15, color, points)
        else:
            marker = MarkerFactory.create_line_marker(f'{ns}_{id_}', n, 0.15, color, points)
        
        return marker


class MarkerFactory:
    """ROS 시각화 마커 생성을 위한 팩토리 클래스"""
    
    @staticmethod
    def create_points_marker(ns, id_, scale, color, points=None):
        """점 마커 생성"""
        marker = Marker()
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.header.frame_id = 'world'
        marker.ns = ns
        marker.id = id_
        marker.lifetime = rospy.Duration(0)
        marker.scale.x = scale
        marker.scale.y = scale
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        
        if points:
            for pt in points:
                marker.points.append(Point(x=pt[0], y=pt[1], z=0.0))
        
        return marker

    @staticmethod
    def create_line_marker(ns, id_, scale, color, points=None):
        """라인 마커 생성"""
        marker = Marker()
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.header.frame_id = 'world'
        marker.ns = ns
        marker.id = id_
        marker.lifetime = rospy.Duration(0)
        marker.scale.x = scale
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.pose.orientation.w = 1.0
        
        if points:
            for pt in points:
                marker.points.append(Point(x=pt[0], y=pt[1], z=0.0))
        
        return marker

    @staticmethod
    def create_text_marker(ns, id_, scale, color, text, position):
        """텍스트 마커 생성"""
        marker = Marker()
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.header.frame_id = 'world'
        marker.ns = ns
        marker.id = id_
        marker.lifetime = rospy.Duration(0)
        marker.text = text
        marker.scale.z = scale
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.pose.position = Point(x=position[0], y=position[1], z=1.0)
        marker.pose.orientation.w = 1.0
        return marker

    @staticmethod
    def create_arrow_marker(ns, id_, scale, color, points):
        """화살표 마커 생성"""
        marker = Marker()
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.header.frame_id = 'world'
        marker.ns = ns
        marker.id = id_
        marker.lifetime = rospy.Duration(0)
        marker.scale.x, marker.scale.y, marker.scale.z = scale
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.pose.orientation.w = 1.0
        
        # 화살표 방향 설정 (끝점 방향)
        num = len(points)
        if num > 2:
            start_idx = -min(max(num, 3), 5)
            marker.points.append(Point(x=points[start_idx][0], y=points[start_idx][1]))
        else:
            marker.points.append(Point(x=points[-2][0], y=points[-2][1]))
        marker.points.append(Point(x=points[-1][0], y=points[-1][1]))
        
        return marker