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

world.tick()

vehicles = world.get_actors().filter('vehicle.tesla.model3')
ego_vehicle = None

for vehicle in vehicles:
    if vehicle.attributes.get('role_name') == "ego_vehicle":
        ego_vehicle = vehicle
        break
print(f"내 차량 정보 : {ego_vehicle}")



# 시작좌표, 목표좌표, 목표 웨이포인트 설정
start_location = ego_vehicle.get_location()
print(start_location)
