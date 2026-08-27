/**
 * Robot pose overlay — footprint polygon + heading (RViz-style), polled from API.
 */
class RobotTracker {
  constructor(map, getOrigin) {
    this.map = map;
    this.getOrigin = getOrigin;
    this.pollMs = 200;
    this.timer = null;
    this.tracking = false;
    this.follow = false;
    this.lastPose = null;

    if (!map.getPane('robotPane')) {
      map.createPane('robotPane');
      map.getPane('robotPane').style.zIndex = 450;
    }
    this.layer = L.layerGroup([], { pane: 'robotPane' }).addTo(map);
    this.footprintPoly = null;
    this.headingLine = null;
    this.centerMarker = null;
  }

  setPollHz(hz) {
    const n = Number(hz);
    this.pollMs = n > 0 ? Math.round(1000 / n) : 200;
    if (this.tracking) {
      this.stop();
      this.start();
    }
  }

  start() {
    if (this.tracking) return;
    this.tracking = true;
    this._tick();
  }

  stop() {
    this.tracking = false;
    if (this.timer) {
      clearTimeout(this.timer);
      this.timer = null;
    }
  }

  setFollow(enabled) {
    this.follow = !!enabled;
  }

  clearLayers() {
    this.layer.clearLayers();
    this.footprintPoly = null;
    this.headingLine = null;
    this.centerMarker = null;
  }

  async _tick() {
    if (!this.tracking) return;
    try {
      const origin = this.getOrigin();
      const pose = await KeepoutApi.robotPose(origin);
      this.updateDisplay(pose);
    } catch (_) {
      /* ignore transient poll errors */
    }
    this.timer = setTimeout(() => this._tick(), this.pollMs);
  }

  updateDisplay(pose) {
    this.lastPose = pose;
    this.clearLayers();

    if (!pose || !pose.available) {
      this._setInfo(pose, 'ROS 未启用');
      return;
    }
    if (!pose.gps_ok) {
      this._setInfo(pose, pose.message || '等待 /gps/fix …');
      return;
    }

    if (pose.footprint_ll && pose.footprint_ll.length >= 3) {
      this.footprintPoly = L.polygon(pose.footprint_ll, {
        color: '#00e676',
        weight: 2,
        fillColor: '#00e676',
        fillOpacity: 0.35,
        interactive: false,
      }).addTo(this.layer);
    }

    if (pose.heading_ll && pose.heading_ll.length === 2) {
      this.headingLine = L.polyline(pose.heading_ll, {
        color: '#ffffff',
        weight: 3,
        opacity: 0.95,
        interactive: false,
      }).addTo(this.layer);
    }

    this.centerMarker = L.circleMarker([pose.lat, pose.lon], {
      radius: 4,
      color: '#ffffff',
      weight: 2,
      fillColor: '#00e676',
      fillOpacity: 1,
      interactive: false,
    }).addTo(this.layer);

    if (this.follow) {
      this.map.panTo([pose.lat, pose.lon], { animate: true, duration: 0.25 });
    }

    this._setInfo(pose);
  }

  _setInfo(pose, extra) {
    const el = document.getElementById('robotInfo');
    if (!el) return;
    if (!pose || !pose.gps_ok) {
      el.textContent = extra || '—';
      return;
    }
    const yaw = pose.yaw_deg != null ? `${pose.yaw_deg.toFixed(1)}°` : '—';
    const mx = pose.map_x != null ? pose.map_x.toFixed(2) : '—';
    const my = pose.map_y != null ? pose.map_y.toFixed(2) : '—';
    const spd =
      pose.speed_mps != null ? `${pose.speed_mps.toFixed(2)} m/s` : '—';
    const gpsTag = pose.gps_ok ? 'GPS ✓' : 'GPS ✗';
    const odomTag = pose.odom_ok ? 'Odom ✓' : 'Odom ✗';
    el.textContent =
      `Lat ${pose.lat.toFixed(6)}\n` +
      `Lon ${pose.lon.toFixed(6)}\n` +
      `Yaw ${yaw}   Speed ${spd}\n` +
      `Map x=${mx}  y=${my} m\n` +
      `${gpsTag}   ${odomTag}`;
  }
}
