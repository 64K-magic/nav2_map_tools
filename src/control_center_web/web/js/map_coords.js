/**
 * Coordinate adapter — tile (WGS84) vs PGM (CRS.Simple / map_server frame).
 * Display coords are always Leaflet [lat, lng]; meaning differs by mode.
 */
const MapCoords = (() => {
  let mode = 'tile';
  /** @type {null | {name,resolution,origin_x,origin_y,origin_yaw,width,height}} */
  let pgmMeta = null;

  function setTileMode() {
    mode = 'tile';
    pgmMeta = null;
  }

  function setPgmMode(meta) {
    mode = 'pgm';
    pgmMeta = {
      ...meta,
      origin_x: meta.origin_x != null ? meta.origin_x : (meta.origin && meta.origin[0]) || 0,
      origin_y: meta.origin_y != null ? meta.origin_y : (meta.origin && meta.origin[1]) || 0,
      origin_yaw: meta.origin_yaw != null ? meta.origin_yaw : (meta.origin && meta.origin[2]) || 0,
    };
  }

  function isPgm() {
    return mode === 'pgm' && !!pgmMeta;
  }

  function getPgmMeta() {
    return pgmMeta;
  }

  function distanceM(a, b) {
    if (isPgm()) {
      const dLat = a[0] - b[0];
      const dLng = a[1] - b[1];
      return Math.hypot(dLat, dLng) * pgmMeta.resolution;
    }
    return KeepoutShapes.haversineM(a, b);
  }

  function mapMetersToDisplay(mx, my) {
    if (!pgmMeta) return [my, mx];
    return [
      (my - pgmMeta.origin_y) / pgmMeta.resolution,
      (mx - pgmMeta.origin_x) / pgmMeta.resolution,
    ];
  }

  function displayToMapMeters(lat, lng) {
    if (!pgmMeta) return { x: lng, y: lat };
    return {
      x: pgmMeta.origin_x + lng * pgmMeta.resolution,
      y: pgmMeta.origin_y + lat * pgmMeta.resolution,
    };
  }

  /** API map-frame figure → display figure for DrawController.loadFigures. */
  function figureFromMapApi(f) {
    const base = {
      figure_type: f.figure_type,
      figure_name: f.figure_name || '',
      id: f.id,
    };
    if (f.figure_type === 'circle') {
      const [lat, lng] = mapMetersToDisplay(f.center_x, f.center_y);
      return { ...base, center_ll: [lat, lng], radius_m: f.radius };
    }
    const vertices_ll = (f.vertices || []).map(([x, y]) => mapMetersToDisplay(x, y));
    return { ...base, vertices_ll };
  }

  /** Display figure → API map-frame payload for PUT keepouts. */
  function figureToMapApi(fig, mapName) {
    const base = {
      figure_type: fig.figure_type,
      figure_name: fig.figure_name || '',
      map_name: mapName,
      center_x: 0,
      center_y: 0,
      radius: 0,
      vertices: [],
    };
    if (fig.figure_type === 'circle') {
      const m = displayToMapMeters(fig.center_ll[0], fig.center_ll[1]);
      return {
        ...base,
        center_x: m.x,
        center_y: m.y,
        radius: fig.radius_m,
      };
    }
    return {
      ...base,
      vertices: fig.vertices_ll.map(([lat, lng]) => {
        const m = displayToMapMeters(lat, lng);
        return [m.x, m.y];
      }),
    };
  }

  function metersToPixels(map, meters, atLatLng) {
    if (isPgm()) {
      if (!meters || meters < 1e-6 || !pgmMeta) return 0;
      // Convert meters → map-image units, then to Leaflet layer pixels at current zoom.
      const d = meters / pgmMeta.resolution;
      const p0 = map.latLngToLayerPoint(atLatLng);
      const p1 = map.latLngToLayerPoint(L.latLng(atLatLng.lat + d, atLatLng.lng));
      return Math.max(0, Math.abs(p1.y - p0.y));
    }
    return InflationPreview.metersToPixelsTile(map, meters, atLatLng);
  }

  function circleDisplayRadius(radiusM) {
    if (isPgm()) return radiusM / pgmMeta.resolution;
    return radiusM;
  }

  function destinationDisplayPoint(centerLl, bearingDeg, distM) {
    if (isPgm()) {
      const br = (bearingDeg * Math.PI) / 180;
      const d = distM / pgmMeta.resolution;
      return [centerLl[0] + d * Math.cos(br), centerLl[1] + d * Math.sin(br)];
    }
    return KeepoutShapes.destinationPoint(centerLl, bearingDeg, distM);
  }

  function robotDisplayLatLng(pose) {
    if (!pose || pose.map_x == null || pose.map_y == null) return null;
    const [lat, lng] = mapMetersToDisplay(pose.map_x, pose.map_y);
    return [lat, lng];
  }

  function footprintToDisplay(footprintMap) {
    if (!footprintMap || !footprintMap.length) return [];
    return footprintMap.map(([x, y]) => mapMetersToDisplay(x, y));
  }

  function headingIconDeg(yawDeg) {
    // Icon default points +Y (map north); ROS yaw CCW from +X.
    return 90 - (yawDeg || 0);
  }

  return {
    setTileMode,
    setPgmMode,
    isPgm,
    getPgmMeta,
    distanceM,
    mapMetersToDisplay,
    displayToMapMeters,
    figureFromMapApi,
    figureToMapApi,
    metersToPixels,
    circleDisplayRadius,
    destinationDisplayPoint,
    robotDisplayLatLng,
    footprintToDisplay,
    headingIconDeg,
  };
})();
