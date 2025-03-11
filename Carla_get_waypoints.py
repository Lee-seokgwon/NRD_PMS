import carla
import carla_util
import sys
sys.path.append(r"D:\CaRLA_0.9.13\PythonAPI\carla") #agent 관련 API호출 위한 환경변수 설정
from agents.navigation.global_route_planner import GlobalRoutePlanner
import carla_util

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


# A* 기반 경로 탐색
route = grp.trace_route(start_location, goal_location)

for waypoint, _ in route:
    world.debug.draw_string(waypoint.transform.location, 'O', draw_shadow=False,
                        color=carla.Color(r=255, g=0, b=0), life_time=120.0,
                        persistent_lines=True)

