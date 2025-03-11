import carla
import carla_util
import sys
sys.path.append(r"D:\CaRLA_0.9.13\PythonAPI\carla") #agent 관련 API호출 위한 환경변수 설정
from agents.navigation.global_route_planner import GlobalRoutePlanner # A*기반 경로 계획 Carla-Python API
from agents.navigation.controller import VehiclePIDController # PID제어 Carla-Python API
import carla_util
import time

# CARLA 서버 연결
client = carla.Client('localhost', 2000)
client.set_timeout(5.0)
world = client.get_world()
map = world.get_map()
sampling_resolution = 2.0
cnt=0

world.tick() #이거 안적으면 월드 불러오고나서 완전히 로드 안된상태에서 액터 찾아버려서 오류 남!!!!! delay같은 놈임.

vehicles = world.get_actors().filter('vehicle.tesla.model3')
ego_vehicle = None

for vehicle in vehicles:
    if vehicle.attributes.get('role_name') == "ego_vehicle":
        ego_vehicle = vehicle
        break
print(f"내 차량 정보 : {ego_vehicle}")

###############################################################
# 현재 차량의 위치를 기준으로 웨이포인트 가져오기
current_waypoint = map.get_waypoint(ego_vehicle.get_location(),project_to_road=True, lane_type=(carla.LaneType.Driving | carla.LaneType.Sidewalk))
print(f"시작점 웨이포인트: {current_waypoint.transform.location}")

# 시작좌표, 목표좌표, 목표 웨이포인트 설정
start_location = ego_vehicle.get_location()
goal_location = carla.Location(x=192.9, y=277.6, z=0)
goal_waypoint = map.get_waypoint(goal_location, project_to_road=True, lane_type=carla.LaneType.Driving)

print(f"도착지 웨이포인트를 찾았습니다: {goal_waypoint.transform.location}")

# 중간 웨이포인트 탐색
waypoint_list = [current_waypoint]
while waypoint_list[-1].transform.location.distance(goal_waypoint.transform.location) > 2.0:
    cnt +=1
    next_wp = waypoint_list[-1].next(2.0)[0]  # 2m 간격으로 다음 웨이포인트 가져오기
    waypoint_list.append(next_wp)
    print(f"추가된 중간 웨이포인트 {cnt} : {next_wp.transform.location}")
#전체 waypoint를 빨간색으로 표시
#carla_util.visualize_waypoints(world, world.get_map(),sampling_resolution)

###############################################################

grp = GlobalRoutePlanner(map, sampling_resolution) #두번째 인자는 웨이포인트 간격 (현재 설정 2)


# A* 기반 경로(웨이포인트) 탐색
route = grp.trace_route(start_location, goal_location)

#웨이포인트 시각화
for waypoint, _ in route:
    world.debug.draw_string(waypoint.transform.location, 'O', draw_shadow=False,
                        color=carla.Color(r=255, g=0, b=0), life_time=120.0,
                        persistent_lines=True)

################################################################

# PID 컨트롤러 설정
args_lateral = {'K_P': 1.95, 'K_D': 0.2, 'K_I': 0.07}
args_longitudinal = {'K_P': 1.0, 'K_D': 0.1, 'K_I': 0.05}
pid_controller = VehiclePIDController(ego_vehicle, args_lateral=args_lateral, args_longitudinal=args_longitudinal)

#웨이포인트 기반 이동
for waypoint, road_option in route:
    target_location = waypoint.transform.location

    while ego_vehicle.get_location().distance(target_location) > 2.0:
        #현재 차량의 상태 업데이트
        vehicle_transform = ego_vehicle.get_transform()

        control = pid_controller.run_step(5.0, waypoint)  # 목표 속도 5m/s
        vehicle.apply_control(control)
        #시뮬레이션 틱(동기모드이면 틱으로 계속 상태 넘겨줘야함!)
        world.tick()
        time.sleep(0.05)