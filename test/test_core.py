import math
from smart_obstacle_avoidance.core import AvoidanceController, make_scan, ray_distance

NO_OBS = []


def test_ray_hits_wall():
    assert abs(ray_distance(5.0, 5.0, 0.0, 10.0, NO_OBS) - 6.0) < 1e-6


def test_ray_hits_obstacle():
    d = ray_distance(2.0, 5.0, 0.0, 10.0, [(5.0, 5.0, 0.5)])
    assert abs(d - 2.5) < 1e-6


def test_range_max_clip():
    assert ray_distance(5.0, 5.0, 0.0, 3.0, NO_OBS) == 3.0


def test_scan_shape():
    ranges, a_min, a_max, inc = make_scan(5, 5, 0, 37, 3.0, NO_OBS)
    assert len(ranges) == 37 and abs(a_min + math.pi / 2) < 1e-9 and abs(inc - math.radians(5)) < 1e-9


def test_forward_when_clear():
    ranges, a, _, inc = make_scan(5, 5, 0, 37, 3.0, NO_OBS)
    d = AvoidanceController().decide(ranges, a, inc)
    assert d.linear > 0 and d.angular == 0


def test_stop_and_turn_when_blocked():
    ranges, a, _, inc = make_scan(5, 5, 0, 37, 3.0, [(6.0, 5.0, 0.5)])   # 0.5 m ahead
    d = AvoidanceController().decide(ranges, a, inc)
    assert d.linear == 0 and d.angular != 0


def test_turns_toward_open_side():
    ranges, a, _, inc = make_scan(5, 5, 0, 37, 3.0, [(6.0, 5.0, 0.5), (5.5, 6.6, 0.5)])  # left side cluttered
    assert AvoidanceController().decide(ranges, a, inc).angular < 0       # turns right


def test_resumes_after_clear():
    c = AvoidanceController()
    blocked, a, _, inc = make_scan(5, 5, 0, 37, 3.0, [(6.0, 5.0, 0.5)])
    clear, *_ = make_scan(5, 5, 0, 37, 3.0, NO_OBS)
    assert c.decide(blocked, a, inc).linear == 0
    assert c.decide(clear, a, inc).linear > 0
