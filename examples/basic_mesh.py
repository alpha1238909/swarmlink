"""Run with: python examples/basic_mesh.py"""

from swarmlink import NodeTelemetry, Position, SimulatedMesh, VehicleKind, Velocity

mesh = SimulatedMesh()

mesh.publish(NodeTelemetry(
    node_id="drone-01", timestamp_s=1.0,
    position=Position(-5.0, 0.0, 10.0), velocity=Velocity(2.0, 0.0, 0.0),
    vehicle_kind=VehicleKind.DRONE, battery_percent=92.0, mission_state="survey",
))
mesh.publish(NodeTelemetry(
    node_id="rover-01", timestamp_s=1.0,
    position=Position(5.0, 0.0, 0.0), velocity=Velocity(-2.0, 0.0, 0.0),
    vehicle_kind=VehicleKind.ROVER, battery_percent=71.0, mission_state="mapping",
))

for alert in mesh.registry.collision_alerts(threshold_m=3.0, horizon_s=5.0):
    print(alert.to_dict())
