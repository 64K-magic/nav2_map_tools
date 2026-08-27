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
    this.headingMarker = null;

    map.on('zoomend', () => {
      if (this.lastPose && this.lastPose.gps_ok) {
        this.updateDisplay(this.lastPose);
      }
    });
  }

  /** Icon pixel size so the arrow fits inside the footprint bbox on screen. */
  _inscribedIconPx(footprintLl) {
    if (!footprintLl || footprintLl.length < 3) return 18;
    const pts = footprintLl.map(([lat, lon]) =>
      this.map.latLngToLayerPoint(L.latLng(lat, lon))
    );
    let minX = Infinity;
    let maxX = -Infinity;
    let minY = Infinity;
    let maxY = -Infinity;
    pts.forEach((p) => {
      minX = Math.min(minX, p.x);
      maxX = Math.max(maxX, p.x);
      minY = Math.min(minY, p.y);
      maxY = Math.max(maxY, p.y);
    });
    const w = maxX - minX;
    const h = maxY - minY;
    // Triangle SVG spans ~83% of icon box; keep inside footprint with margin.
    return Math.max(10, Math.min(w, h) * 0.62);
  }

  static _bearingDeg(from, to) {
    const lat1 = (from[0] * Math.PI) / 180;
    const lat2 = (to[0] * Math.PI) / 180;
    const dLon = ((to[1] - from[1]) * Math.PI) / 180;
    const y = Math.sin(dLon) * Math.cos(lat2);
    const x =
      Math.cos(lat1) * Math.sin(lat2) -
      Math.sin(lat1) * Math.cos(lat2) * Math.cos(dLon);
    return ((Math.atan2(y, x) * 180) / Math.PI + 360) % 360;
  }

  static _headingSvg() {
    return (
      '<svg viewBox="0 0 48 48" xmlns="http://www.w3.org/2000/svg" aria-hidden="true">' +
      '<polygon points="24,2 44,42 24,30 4,42" fill="#ff2d4a"/>' +
      '<polygon points="24,2 44,42 24,30" fill="#ff5569" opacity="0.9"/>' +
      '<polygon points="24,2 4,42 24,30" fill="#d81835"/>' +
      '<line x1="24" y1="2" x2="24" y2="30" stroke="#ffc8d0" stroke-width="1.2"/>' +
      '<line x1="24" y1="2" x2="44" y2="42" stroke="#ffc8d0" stroke-width="0.9"/>' +
      '<line x1="24" y1="2" x2="4" y2="42" stroke="#ffc8d0" stroke-width="0.9"/>' +
      '</svg>'
    );
  }

  _createHeadingIcon(bearingDeg, sizePx) {
    const s = Math.round(sizePx || 18);
    const half = s / 2;
    return L.divIcon({
      className: 'robot-heading-icon',
      html:
        `<div class="robot-heading-inner" style="width:${s}px;height:${s}px;transform:rotate(${bearingDeg}deg)">` +
        `${RobotTracker._headingSvg()}</div>`,
      iconSize: [s, s],
      iconAnchor: [half, half],
    });
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
    this.headingMarker = null;
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

    const iconPx = this._inscribedIconPx(pose.footprint_ll);

    if (pose.heading_ll && pose.heading_ll.length === 2) {
      const bearing = RobotTracker._bearingDeg(pose.heading_ll[0], pose.heading_ll[1]);
      this.headingMarker = L.marker([pose.lat, pose.lon], {
        icon: this._createHeadingIcon(bearing, iconPx),
        interactive: false,
        zIndexOffset: 500,
      }).addTo(this.layer);
    } else if (pose.yaw_deg != null) {
      const yawRad = (pose.yaw_deg * Math.PI) / 180;
      const bearing =
        ((Math.atan2(Math.cos(yawRad), Math.sin(yawRad)) * 180) / Math.PI + 360) % 360;
      this.headingMarker = L.marker([pose.lat, pose.lon], {
        icon: this._createHeadingIcon(bearing, iconPx),
        interactive: false,
        zIndexOffset: 500,
      }).addTo(this.layer);
    }

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
