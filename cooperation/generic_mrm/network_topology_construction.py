#!/usr/bin/env python3

import rospy
import math
import itertools
from collections import defaultdict

class NetworkTopologyConstruction:
    def __init__(self, ros_manager):
        self.ros_manager = ros_manager
        self.my_vehicle_id = ros_manager.my_vehicle_id
        
        # 토폴로지 상태
        self.generation_map = {}  # {generation: [vehicle_ids]}
        self.vehicle_info = {}    # {vehicle_id: {generation, parents, children}}
        self.MAX_GENERATION = 5
        self.COMM_RANGE = 35.0  
        
        # 로컬 상태
        self.assigned_vehicles = set()  # 이미 할당된 차량들
        self.topology_constructed = False
        
        rospy.loginfo(f"NTC initialized for {self.my_vehicle_id}")
    
    def execute(self):
        """메인 실행 함수 - 모든 차량이 동일한 코드 실행, 상황별 판단"""
        all_vehicles = self.ros_manager.get_all_vehicles()
        if not all_vehicles:
            return
        
        # 0. 다른 차량들의 children 정보를 보고 내 generation 유추 (새로 추가!)
        if not self.topology_constructed:
            self._infer_my_generation_from_parents(all_vehicles)
            
        # 1. 내가 fallback이라면 토폴로지 시작
        if self.ros_manager.am_i_fallback() and not self.topology_constructed:
            rospy.loginfo(f"{self.my_vehicle_id}: I am fallback, starting topology construction")
            self._start_as_fallback()
            return
        
        # 2. fallback이 토폴로지를 시작했는지 확인
        if not self._is_topology_active():
            return
        
        # 3. 내가 assigner 역할을 해야 하는지 확인
        if self._should_i_assign():
            rospy.loginfo(f"{self.my_vehicle_id}: My turn to assign children")
            self._assign_my_children()
        
        # 4. 시각화 업데이트 (ego만 - 내부에서 판단)
        if self.my_vehicle_id == 'ego' and self.topology_constructed:
            self._update_visualization()
    
    def _start_as_fallback(self):
        """Fallback 차량으로 토폴로지 시작"""
        rospy.loginfo(f"{self.my_vehicle_id}: Starting as fallback (Generation 0)")
        
        # 내 정보 초기화
        self.generation_map[0] = [self.my_vehicle_id]
        self.vehicle_info[self.my_vehicle_id] = {
            'generation': 0,
            'parents': [],
            'children': []
        }
        self.assigned_vehicles.add(self.my_vehicle_id)
        
        # 자식 할당
        all_vehicles = self.ros_manager.get_all_vehicles()
        assigned_children = self._assign_children(self.my_vehicle_id, all_vehicles, 0)
        
        # 내 상태 업데이트
        self.ros_manager.update_my_topology_info(
            generation=0,
            is_assigner=len(assigned_children) > 0,
            children=assigned_children,
            max_gen=1 if assigned_children else 0
        )
        
        self.topology_constructed = True
        rospy.loginfo(f"{self.my_vehicle_id}: Fallback assignment completed")
    
    def _is_topology_active(self):
        """토폴로지 구성이 시작되었는지 확인"""
        all_vehicles = self.ros_manager.get_all_vehicles()
        
        for vehicle_id, data in all_vehicles.items():
            if hasattr(data, 'max_generation') and data.max_generation > 0:
                return True
        return False
    
    def _should_i_assign(self):
        """내가 자식을 할당해야 하는 차례인지 확인"""
        # 이미 내가 할당을 완료했다면 false
        if self.topology_constructed:
            return False
        
        all_vehicles = self.ros_manager.get_all_vehicles()
        my_data = all_vehicles.get(self.my_vehicle_id)
        
        if not my_data:
            return False
        
        # 내 generation이 설정되지 않았다면 false (255는 미할당)
        if not hasattr(my_data, 'generation_level') or my_data.generation_level == 255:
            return False
        
        my_generation = my_data.generation_level
        
        # 내가 이미 assigner로 설정되었고 children이 있다면 false (이미 완료)
        if hasattr(my_data, 'is_assigner') and my_data.is_assigner:
            if hasattr(my_data, 'my_children') and len(my_data.my_children) > 0:
                return False
        
        # 내 generation의 모든 차량들이 준비되었는지 확인
        same_gen_vehicles = []
        for vehicle_id, data in all_vehicles.items():
            if (hasattr(data, 'generation_level') and 
                data.generation_level == my_generation and
                data.generation_level != 255):  # 미할당 제외
                same_gen_vehicles.append(vehicle_id)
        
        # 같은 generation의 차량이 모두 데이터를 받았는지 확인
        if len(same_gen_vehicles) == 0:
            return False
        
        # 아직 할당되지 않은 차량들이 있는지 확인
        unassigned_count = 0
        for vehicle_id, data in all_vehicles.items():
            if (not hasattr(data, 'generation_level') or 
                data.generation_level == 255):  # 255는 미할당
                unassigned_count += 1
        
        # 할당할 차량이 없으면 false
        if unassigned_count == 0:
            return False
        
        rospy.loginfo(f"{self.my_vehicle_id}: Should I assign? My gen={my_generation}, "
                     f"same_gen_vehicles={len(same_gen_vehicles)}, unassigned={unassigned_count}")
        
        return True
    
    def _assign_my_children(self):
        """내가 담당하는 자식 차량들 할당"""
        all_vehicles = self.ros_manager.get_all_vehicles()
        my_data = all_vehicles.get(self.my_vehicle_id)
        
        if not my_data:
            return
        
        my_generation = my_data.generation_level
        rospy.loginfo(f"{self.my_vehicle_id}: Assigning children for generation {my_generation}")
        
        # 자식 할당
        assigned_children = self._assign_children(self.my_vehicle_id, all_vehicles, my_generation)
        
        if assigned_children:
            # generation_map 업데이트 (local tracking용)
            next_gen = my_generation + 1
            if next_gen not in self.generation_map:
                self.generation_map[next_gen] = []
            
            for child_id in assigned_children:
                if child_id not in self.generation_map[next_gen]:
                    self.generation_map[next_gen].append(child_id)
                
                # vehicle_info 업데이트 (local tracking용)
                self.vehicle_info[child_id] = {
                    'generation': next_gen,
                    'parents': [self.my_vehicle_id],
                    'children': []
                }
            
            # 내 정보 업데이트
            if self.my_vehicle_id not in self.vehicle_info:
                self.vehicle_info[self.my_vehicle_id] = {
                    'generation': my_generation,
                    'parents': [],
                    'children': []
                }
            self.vehicle_info[self.my_vehicle_id]['children'] = assigned_children
        
        # CoopData 메시지 업데이트
        current_max_gen = max([data.max_generation for data in all_vehicles.values() 
                              if hasattr(data, 'max_generation')], default=0)
        new_max_gen = max(current_max_gen, my_generation + (1 if assigned_children else 0))
        
        self.ros_manager.update_my_topology_info(
            generation=my_generation,
            is_assigner=len(assigned_children) > 0,
            children=assigned_children,
            max_gen=new_max_gen
        )
        
        self.topology_constructed = True
        rospy.loginfo(f"{self.my_vehicle_id}: Assignment completed, children: {assigned_children}")
    
    def _update_visualization(self):
        """시각화 업데이트 (ego 전용)"""
        all_vehicles = self.ros_manager.get_all_vehicles()
        
        # 전체 토폴로지 정보 수집
        complete_generation_map = {}
        complete_vehicle_info = {}
        
        for vehicle_id, data in all_vehicles.items():
            if hasattr(data, 'generation_level') and data.generation_level != 255:  # 255는 미할당
                gen = data.generation_level
                if gen not in complete_generation_map:
                    complete_generation_map[gen] = []
                complete_generation_map[gen].append(vehicle_id)
                
                # 부모 정보 추론 (children 정보로부터)
                parents = []
                for other_id, other_data in all_vehicles.items():
                    if (hasattr(other_data, 'my_children') and 
                        vehicle_id in other_data.my_children):
                        parents.append(other_id)
                
                children = getattr(data, 'my_children', [])
                
                complete_vehicle_info[vehicle_id] = {
                    'generation': gen,
                    'parents': parents,
                    'children': children
                }
        
        if complete_generation_map and complete_vehicle_info:
            self.ros_manager.publish_topology_visualization(
                complete_generation_map, complete_vehicle_info
            )
    
    def _construct_topology(self):
        """토폴로지 구성 (기존 알고리즘)"""
        rospy.loginfo(f"{self.my_vehicle_id}: Starting topology construction")
        
        all_vehicles = self.ros_manager.get_all_vehicles()
        if len(all_vehicles) < 2:
            rospy.logwarn("Not enough vehicles for topology construction")
            return
        
        # 알고리즘 실행
        self._reset_topology()
        self._generation_assignment_algorithm(self.my_vehicle_id, all_vehicles)
        self._assign_group_responsibilities()
        self._print_topology()
        
        # 내 정보 업데이트
        if self.my_vehicle_id in self.vehicle_info:
            info = self.vehicle_info[self.my_vehicle_id]
            self.ros_manager.update_my_topology_info(
                generation=info['generation'],
                is_assigner=len(info['children']) > 0,
                children=info['children'],
                max_gen=max(self.generation_map.keys()) if self.generation_map else 0
            )
        
        # 시각화 퍼블리시
        self.ros_manager.publish_topology_visualization(self.generation_map, self.vehicle_info)
        
        self.topology_constructed = True
        rospy.loginfo("Topology construction completed")
    
    def _reset_topology(self):
        """토폴로지 리셋"""
        self.generation_map = {}
        self.vehicle_info = {}
        self.assigned_vehicles = set()
    
    def _generation_assignment_algorithm(self, fallback_id, all_vehicles):
        """Generation Assignment Algorithm"""
        rospy.loginfo("Running Generation Assignment Algorithm")
        
        # Step 1: Initialize fallback vehicle
        self.vehicle_info[fallback_id] = {
            'generation': 0,
            'parents': [],
            'children': []
        }
        self.generation_map[0] = [fallback_id]
        self.assigned_vehicles.add(fallback_id)
        
        # Step 2: Assign generations
        for generation in range(self.MAX_GENERATION):
            if generation not in self.generation_map:
                break
                
            current_gen_vehicles = self.generation_map[generation]
            rospy.loginfo(f"Processing generation {generation}: {current_gen_vehicles}")
            
            for vehicle_id in current_gen_vehicles:
                # 이 차량이 assigner 역할
                assigned_children = self._assign_children(vehicle_id, all_vehicles, generation)
                
                if assigned_children:
                    next_gen = generation + 1
                    if next_gen not in self.generation_map:
                        self.generation_map[next_gen] = []
                    
                    for child_id in assigned_children:
                        self.generation_map[next_gen].append(child_id)
                        rospy.loginfo(f"Assigned {child_id} to generation {next_gen}, parent: {vehicle_id}")
    
    def _assign_children(self, assigner_id, all_vehicles, current_generation):
        """특정 assigner가 자식 차량들 할당 - 원래 필터링 순서로 복원"""
        if assigner_id not in all_vehicles:
            return []
        
        assigner_data = all_vehicles[assigner_id]
        rospy.loginfo(f"\n=== Assignment process for {assigner_id} (Gen {current_generation}) ===")
        
        # 1. Localization Validity: 통신 범위 내 차량들
        neighbors = self.ros_manager.find_neighbors_in_range(assigner_id, self.COMM_RANGE)
        rospy.loginfo(f"Step 1 - Localization Validity: Found {len(neighbors)} neighbors in {self.COMM_RANGE}m range")
        for neighbor_id, neighbor_data, distance in neighbors:
            rospy.loginfo(f"  {neighbor_id}: distance={distance:.1f}m")
        
        # 2. Redundancy Avoidance: CoopData의 generation_level 확인
        available_candidates = []
        for neighbor_id, neighbor_data, distance in neighbors:
            # CoopData에서 generation_level 확인 (255는 미할당)
            neighbor_gen = getattr(neighbor_data, 'generation_level', 255)
            if neighbor_gen == 255:  # 아직 할당되지 않음
                available_candidates.append((neighbor_id, neighbor_data, distance))
                rospy.loginfo(f"  {neighbor_id} is available (gen={neighbor_gen})")
            else:
                rospy.loginfo(f"  {neighbor_id} already assigned (gen={neighbor_gen})")
        
        rospy.loginfo(f"Step 2 - Redundancy Avoidance: {len(available_candidates)} available candidates")
        for candidate_id, candidate_data, distance in available_candidates:
            rospy.loginfo(f"  {candidate_id}: distance={distance:.1f}m")
        
        if not available_candidates:
            rospy.loginfo("No available candidates after redundancy filtering")
            return []
        
        # 3. Coverage Diversity Maximization
        if len(available_candidates) <= 2:
            # 후보가 2개 이하면 모두 선택
            selected = available_candidates
            rospy.loginfo(f"Step 3 - Coverage Diversity: Selected all {len(selected)} candidates (≤2)")
        else:
            # 모든 2개 조합에 대해 diversity 계산
            rospy.loginfo(f"Step 3 - Coverage Diversity: Evaluating all combinations of 2 from {len(available_candidates)} candidates")
            
            best_combination = None
            best_diversity_score = -1
            combination_scores = []
            
            for combination in itertools.combinations(available_candidates, 2):
                diversity_score = self._calculate_combination_diversity(
                    assigner_data, combination[0], combination[1]
                )
                combination_scores.append((combination, diversity_score))
                
                if diversity_score > best_diversity_score:
                    best_diversity_score = diversity_score
                    best_combination = combination
            
            # 모든 조합 점수 출력 (거리 정보 포함)
            rospy.loginfo("Combination diversity scores:")
            for (cand1, cand2), score in sorted(combination_scores, key=lambda x: x[1], reverse=True):
                avg_dist = (cand1[2] + cand2[2]) / 2.0
                rospy.loginfo(f"  {cand1[0]}-{cand2[0]}: {score:.3f} (avg_dist: {avg_dist:.1f}m)")
            
            # 최고 점수와 비슷한 조합들 중에서 거리가 가까운 것 선택
            top_threshold = best_diversity_score * 0.95  # 5% 이내 차이면 거리로 결정
            close_combinations = [(comb, score) for comb, score in combination_scores if score >= top_threshold]
            
            if len(close_combinations) > 1:
                # 거리 기준으로 재선택
                selected_comb = min(close_combinations, key=lambda x: (x[0][0][2] + x[0][1][2]) / 2.0)
                selected = list(selected_comb[0])
                rospy.loginfo(f"Multiple top combinations found, selected by distance: {selected[0][0]}-{selected[1][0]}")
            else:
                selected = list(best_combination) if best_combination else available_candidates[:2]
                rospy.loginfo(f"Best combination: {selected[0][0]}-{selected[1][0]} (score: {best_diversity_score:.3f})")
        
        # 선택된 차량들 정보 업데이트
        assigned_children = []
        next_generation = current_generation + 1
        
        for child_id, child_data, distance in selected:
            # 자식 정보 설정
            self.vehicle_info[child_id] = {
                'generation': next_generation,
                'parents': [assigner_id],
                'children': []
            }
            
            # 부모 정보 업데이트
            if assigner_id not in self.vehicle_info:
                self.vehicle_info[assigner_id] = {
                    'generation': current_generation,
                    'parents': [],
                    'children': []
                }
            self.vehicle_info[assigner_id]['children'].append(child_id)
            
            # 할당된 차량으로 마킹 (local tracking용)
            self.assigned_vehicles.add(child_id)
            assigned_children.append(child_id)
            
            rospy.loginfo(f"Final selection: {child_id} (distance: {distance:.1f}m, generation: {next_generation})")
        
        rospy.loginfo("=== End of assignment process ===\n")
        return assigned_children
    

    def _infer_my_generation_from_parents(self, all_vehicles):
        """다른 차량들의 children 정보를 보고 내 generation 유추하고 설정"""
        my_current_data = all_vehicles.get(self.my_vehicle_id)
        if not my_current_data:
            return
        
        # 이미 generation이 설정되어 있으면 return
        if (hasattr(my_current_data, 'generation_level') and 
            my_current_data.generation_level != 255):
            return
        
        # 모든 차량을 확인해서 내가 누군가의 자식인지 찾기
        for vehicle_id, data in all_vehicles.items():
            if vehicle_id == self.my_vehicle_id:
                continue
                
            # 이 차량이 나를 자식으로 가지고 있는지 확인
            if (hasattr(data, 'my_children') and 
                hasattr(data, 'generation_level') and
                data.generation_level != 255 and
                len(data.my_children) > 0 and
                self.my_vehicle_id in data.my_children):
                
                # 내가 이 차량의 자식이다!
                parent_generation = data.generation_level
                my_generation = parent_generation + 1
                
                rospy.loginfo(f"{self.my_vehicle_id}: Discovered I'm child of {vehicle_id}! "
                            f"Parent gen:{parent_generation} -> Setting my gen:{my_generation}")
                
                # 내 정보 업데이트
                self.ros_manager.update_my_topology_info(
                    generation=my_generation,
                    is_assigner=False,  # 아직 자식 할당 안함
                    children=[],
                    max_gen=my_generation
                )
                
                rospy.loginfo(f"{self.my_vehicle_id}: Now I know my generation is {my_generation}")
                return  # 찾았으니 종료
    
    def _calculate_combination_diversity(self, assigner_data, candidate1, candidate2):
        """두 후보의 조합 diversity 점수 계산 - 상세 로그 추가"""
        candidate1_id, candidate1_data, dist1 = candidate1
        candidate2_id, candidate2_data, dist2 = candidate2
        
        # 1. Perception overlap 계산 (1 - overlap)
        overlap = self._calculate_perception_overlap(candidate1_data, candidate2_data)
        non_overlap_score = 1.0 - overlap
        
        # 2. Assigner를 바라보는 방향의 각도 차이 (1 - cos(angle_diff))
        angle1 = self._calculate_angle_to_assigner(assigner_data, candidate1_data)
        angle2 = self._calculate_angle_to_assigner(assigner_data, candidate2_data)
        
        angle_diff = abs(angle1 - angle2)
        angle_diff = min(angle_diff, 2*math.pi - angle_diff)  # 최소 각도
        
        # 1 - cos(angle_diff): 각도 차이가 클수록 (반대 방향일수록) 높은 점수
        direction_diversity = 1.0 - math.cos(angle_diff)
        
        # 거리 기반 보정 강화
        avg_distance = (dist1 + dist2) / 2.0
        distance_penalty = avg_distance / 50.0  # 통신 범위 대비 거리 비율
        distance_factor = max(0.1, 1.0 - distance_penalty)  # 거리가 가까울수록 높은 점수
        
        # 최종 diversity 점수 (거리 요소를 더 강하게 반영)
        base_diversity = non_overlap_score * 0.3 + direction_diversity * 0.3
        diversity_score = base_diversity + distance_factor * 0.4  # 거리 비중 40%로 증가
        
        rospy.logdebug(f"    Detailed diversity for {candidate1_id}-{candidate2_id}:")
        rospy.logdebug(f"      Distances: {dist1:.1f}m, {dist2:.1f}m (avg: {avg_distance:.1f}m)")
        rospy.logdebug(f"      Distance penalty: {distance_penalty:.3f} → distance_factor: {distance_factor:.3f}")
        rospy.logdebug(f"      Perception overlap: {overlap:.3f} → non_overlap_score: {non_overlap_score:.3f}")
        rospy.logdebug(f"      Angle difference: {math.degrees(angle_diff):.1f}° → direction_diversity: {direction_diversity:.3f}")
        rospy.logdebug(f"      Base diversity: {base_diversity:.3f}, Final score: {diversity_score:.3f}")
        
        return diversity_score
    
    def _calculate_perception_overlap(self, candidate1_data, candidate2_data):
        """두 차량의 perception 반경 overlap 계산"""
        # Perception radius 가져오기 (0이면 기본값 20.0)
        radius1 = getattr(candidate1_data, 'perception_radius', 20.0)
        radius2 = getattr(candidate2_data, 'perception_radius', 20.0)
        if radius1 <= 0:
            radius1 = 20.0
        if radius2 <= 0:
            radius2 = 20.0
        
        # 두 원의 중심 간 거리
        dx = candidate1_data.pose.position.x - candidate2_data.pose.position.x
        dy = candidate1_data.pose.position.y - candidate2_data.pose.position.y
        distance = math.sqrt(dx**2 + dy**2)
        
        # 겹치지 않으면 overlap = 0
        if distance >= radius1 + radius2:
            return 0.0
        
        # 완전히 포함되면 작은 원의 넓이 / 큰 원의 넓이
        if distance <= abs(radius1 - radius2):
            smaller_area = math.pi * min(radius1, radius2)**2
            larger_area = math.pi * max(radius1, radius2)**2
            return smaller_area / larger_area
        
        # 부분적 겹침: 교집합 넓이 계산 (복잡한 기하학적 계산)
        # 단순화: 거리 기반 근사치
        overlap_ratio = 1.0 - (distance / (radius1 + radius2))
        return max(0.0, overlap_ratio)
    
    def _calculate_angle_to_assigner(self, assigner_data, candidate_data):
        """Candidate가 assigner를 바라보는 방향 각도"""
        dx = assigner_data.pose.position.x - candidate_data.pose.position.x
        dy = assigner_data.pose.position.y - candidate_data.pose.position.y
        return math.atan2(dy, dx)
    
    def _assign_group_responsibilities(self):
        """Group Responsibility Assignment (단순화 버전)"""
        rospy.loginfo("Assigning group responsibilities")
        
        # 현재는 단순히 같은 세대 차량들이 협력한다고 가정
        for generation in range(1, self.MAX_GENERATION):
            if generation not in self.generation_map:
                continue
                
            generation_vehicles = self.generation_map[generation]
            rospy.loginfo(f"Generation {generation} vehicles: {generation_vehicles} will cooperate")
    
    def _print_topology(self):
        """구성된 토폴로지 출력"""
        rospy.loginfo("=== Network Topology Results ===")
        
        for generation in sorted(self.generation_map.keys()):
            vehicles = self.generation_map[generation]
            rospy.loginfo(f"Generation {generation}: {vehicles}")
            
            for vehicle_id in vehicles:
                if vehicle_id in self.vehicle_info:
                    info = self.vehicle_info[vehicle_id]
                    rospy.loginfo(f"  {vehicle_id}:")
                    rospy.loginfo(f"    Parents: {info['parents']}")
                    rospy.loginfo(f"    Children: {info['children']}")
        
        rospy.loginfo("=== End of Topology Results ===")