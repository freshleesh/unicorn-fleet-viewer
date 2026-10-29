import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock

import numpy as np
import pytest
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

spec = importlib.util.spec_from_file_location('lap_timer', Path(__file__).parents[1] / 'scripts/lap_timer.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)

@pytest.fixture
def timer(tmp_path, monkeypatch):
    monkeypatch.setattr(m, 'LINE_FILE', str(tmp_path / 'lines.yaml'))
    t = m.LapTimer.__new__(m.LapTimer)
    t.track = NS(length=10., s=np.arange(0., 10., .1),
                 tangent=np.tile([1., 0.], (100, 1)), pts=np.column_stack([np.arange(0.,10.,.1),np.zeros(100)]),
                 half_width=np.ones(100), project=lambda x,y: int(round(x*10)) % 100)
    t.lines = {}; t.map_yaml='/track.yaml'; t.log=Mock(); t.marker_pub=Mock(); t.state_pub=Mock()
    t.cars={}; t.race=m.Race(10.)
    return t

def test_independent_lines_and_directions(timer):
    timer.set_line(0.,0.,0.,'nuc1')
    timer.set_line(5.,0.,np.pi,'nuc4')
    assert timer.offset('nuc1', 10) == pytest.approx(1.)
    assert timer.offset('nuc4', 40) == pytest.approx(1.)
    assert timer.line_for('nuc5') is None
    timer.race.start(1,0.)
    timer.race.go({'nuc1':timer.offset('nuc1',99),'nuc4':timer.offset('nuc4',51)})
    # First crossing is not a completed lap for either starting position/direction.
    for car in ('nuc1','nuc4'):
        timer.race.advance(car,.2,3.,3.2)
        assert timer.race.cars[car].laps == 0
    timer.race.advance('nuc4',10.,3.2,13.2)
    timer.race.advance('nuc1',10.,3.2,14.2)
    assert timer.race.phase == 'finished'
    assert timer.race.standings() == {'nuc4':1,'nuc1':2}

def test_only_configured_fresh_cars_enter_and_marker_ids_unique(timer, monkeypatch):
    timer.set_line(0.,0.,0.,'nuc1');timer.set_line(5.,0.,0.,'nuc4')
    timer.cars={'nuc1':dict(idx=99,t=3.), 'nuc4':dict(idx=49,t=3.), 'nuc5':dict(idx=30,t=3.)}
    timer.race.start(2,0.);monkeypatch.setattr(m.time,'monotonic',lambda:3.1);timer.tick()
    assert set(timer.race.cars)=={'nuc1','nuc4'}
    state=json.loads(timer.state_pub.publish.call_args.args[0].data)
    assert state['start_mode']=='individual' and not state['cars']['nuc5']['in_race']
    markers=timer.marker_pub.publish.call_args.args[0].markers
    assert len({(x.ns,x.id) for x in markers})==len(markers)
    labels=[x.text for x in markers if x.type==m.Marker.TEXT_VIEW_FACING and x.action==m.Marker.ADD]
    assert sorted(labels)==['nuc1 START / FINISH','nuc4 START / FINISH']

def test_save_clear_and_active_race_protection(timer):
    timer.set_line(2.,0.,0.,'nuc1');timer.set_line(7.,0.,0.,'nuc4')
    data=m.yaml.safe_load(Path(m.LINE_FILE).read_text())
    assert data['map']=='/track.yaml' and set(data['lines'])=={'nuc1','nuc4'}
    timer.race.start(3,0.)
    msg=PoseStamped();msg.header.frame_id='map';msg.pose.position.x=4.;msg.pose.orientation.w=1.
    timer.on_line(msg,'nuc1');timer.on_command(String(data='clear_lines'))
    assert timer.lines['nuc1']['x']==2.
    timer.on_command(String(data='reset'));timer.on_command(String(data='clear_lines'))
    assert not timer.lines and not m.yaml.safe_load(Path(m.LINE_FILE).read_text())['lines']

def test_shared_compatibility_and_empty_race(timer,monkeypatch):
    timer.set_line(0.,0.,0.)
    assert timer.line_for('nuc4') is not None
    timer.set_line(5.,0.,0.,'nuc1')
    assert timer.line_for('nuc4') is None
    timer.race.start(1,0.);monkeypatch.setattr(m.time,'monotonic',lambda:3.1);timer.tick()
    assert timer.race.phase=='idle'

def test_reload_and_wrong_map_rejection(timer, monkeypatch):
    timer.set_line(2.,0.,0.,'nuc1');timer.set_line(7.,0.,np.pi,'nuc4')
    node=Mock();node.declare_parameter.return_value.value='/track.yaml'
    monkeypatch.setattr(m.rclpy,'create_node',lambda name:node)
    monkeypatch.setattr(m,'Track',lambda path:timer.track)
    restored=m.LapTimer()
    assert restored.offset('nuc4',60)==pytest.approx(1.)
    assert set(restored.lines)=={'nuc1','nuc4'}
    node.declare_parameter.return_value.value='/different.yaml'
    assert not m.LapTimer().lines


def test_legacy_line_file_loads_shared(timer, monkeypatch):
    Path(m.LINE_FILE).write_text('x: 2.0\ny: 0.0\nyaw: 0.0\n')
    node=Mock();node.declare_parameter.return_value.value='/track.yaml'
    monkeypatch.setattr(m.rclpy,'create_node',lambda name:node)
    monkeypatch.setattr(m,'Track',lambda path:timer.track)
    restored=m.LapTimer()
    assert restored.offset('nuc4',30)==pytest.approx(1.)
