/**
 * Keepout shape models + Leaflet layer helpers + geometry transforms.
 * Vertices stored as [lat, lon] (Leaflet LatLng order).
 */
const KeepoutShapes = (() => {
  const STYLE = {
    color: '#ff4444',
    weight: 2,
    fillColor: '#ff4444',
    fillOpacity: 0.25,
  };
  const LINE_STYLE = { color: '#ff4444', weight: 3, fill: false };
  const SELECT_STYLE = { color: '#ffaa00', weight: 3, fillColor: '#ffaa00', fillOpacity: 0.2 };
  const SELECT_LINE = { color: '#ffaa00', weight: 3, fill: false };

  function createLayer(figure) {
    const t = figure.figure_type;
    if (t === 'circle') {
      const [lat, lon] = figure.center_ll;
      return L.circle([lat, lon], { ...STYLE, radius: figure.radius_m || 1 });
    }
    if (t === 'line') {
      return L.polyline(figure.vertices_ll, LINE_STYLE);
    }
    return L.polygon(figure.vertices_ll, STYLE);
  }

  function applyStyle(layer, figure, selected) {
    if (!layer || !layer.setStyle) return;
    if (figure.figure_type === 'line') {
      layer.setStyle(selected ? SELECT_LINE : LINE_STYLE);
    } else {
      layer.setStyle(selected ? SELECT_STYLE : STYLE);
    }
  }

  function toWgs84Payload(figure) {
    const base = {
      figure_type: figure.figure_type,
      figure_name: figure.figure_name || '',
    };
    if (figure.figure_type === 'circle') {
      return {
        ...base,
        center_ll: figure.center_ll,
        radius_m: figure.radius_m,
      };
    }
    return {
      ...base,
      vertices_ll: figure.vertices_ll,
    };
  }

  function rectCorners(a, b) {
    const latMin = Math.min(a[0], b[0]);
    const latMax = Math.max(a[0], b[0]);
    const lonMin = Math.min(a[1], b[1]);
    const lonMax = Math.max(a[1], b[1]);
    return [
      [latMax, lonMin],
      [latMax, lonMax],
      [latMin, lonMax],
      [latMin, lonMin],
    ];
  }

  function haversineM(a, b) {
    const R = 6378137;
    const toR = Math.PI / 180;
    const dLat = (b[0] - a[0]) * toR;
    const dLon = (b[1] - a[1]) * toR;
    const la1 = a[0] * toR;
    const la2 = b[0] * toR;
    const h =
      Math.sin(dLat / 2) ** 2 +
      Math.cos(la1) * Math.cos(la2) * Math.sin(dLon / 2) ** 2;
    return 2 * R * Math.asin(Math.min(1, Math.sqrt(h)));
  }

  function fromApiFigure(f) {
    return {
      figure_type: f.figure_type,
      figure_name: f.figure_name || '',
      vertices_ll: f.vertices_ll ? f.vertices_ll.map((p) => [p[0], p[1]]) : [],
      center_ll: f.center_ll ? [f.center_ll[0], f.center_ll[1]] : null,
      radius_m: f.radius_m != null ? f.radius_m : null,
      id: f.id,
    };
  }

  function centroid(figure) {
    if (figure.figure_type === 'circle') {
      return figure.center_ll.slice();
    }
    const pts = figure.vertices_ll;
    if (!pts.length) return [0, 0];
    // Line: geometric mid of first–last endpoints
    if (figure.figure_type === 'line' && pts.length >= 2) {
      const a = pts[0];
      const b = pts[pts.length - 1];
      return [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2];
    }
    let lat = 0;
    let lon = 0;
    pts.forEach((p) => {
      lat += p[0];
      lon += p[1];
    });
    return [lat / pts.length, lon / pts.length];
  }

  /** Map container helpers for rotate/scale without heavy geodesic math. */
  function toLayerPts(map, latlngs) {
    return latlngs.map((ll) => map.latLngToLayerPoint(L.latLng(ll[0], ll[1])));
  }

  function fromLayerPts(map, pts) {
    return pts.map((p) => {
      const ll = map.layerPointToLatLng(p);
      return [ll.lat, ll.lng];
    });
  }

  function translateFigure(figure, dLat, dLon) {
    if (figure.figure_type === 'circle') {
      figure.center_ll = [figure.center_ll[0] + dLat, figure.center_ll[1] + dLon];
      return;
    }
    figure.vertices_ll = figure.vertices_ll.map((p) => [p[0] + dLat, p[1] + dLon]);
  }

  /**
   * Rotate figure around centroid by delta radians (map layer-pixel space).
   */
  function rotateFigure(map, figure, deltaRad) {
    const c = centroid(figure);
    const cPt = map.latLngToLayerPoint(L.latLng(c[0], c[1]));
    const cos = Math.cos(deltaRad);
    const sin = Math.sin(deltaRad);

    function rotPt(pt) {
      const dx = pt.x - cPt.x;
      const dy = pt.y - cPt.y;
      return L.point(cPt.x + dx * cos - dy * sin, cPt.y + dx * sin + dy * cos);
    }

    if (figure.figure_type === 'circle') {
      // circle is rotationally invariant
      return;
    }
    const pts = toLayerPts(map, figure.vertices_ll).map(rotPt);
    figure.vertices_ll = fromLayerPts(map, pts);
  }

  /**
   * Uniform scale about centroid using layer pixels; for circle updates radius_m.
   */
  function scaleFigure(map, figure, factor) {
    if (factor < 0.05) factor = 0.05;
    const c = centroid(figure);
    if (figure.figure_type === 'circle') {
      figure.radius_m = Math.max(0.5, figure.radius_m * factor);
      return;
    }
    const cPt = map.latLngToLayerPoint(L.latLng(c[0], c[1]));
    const pts = toLayerPts(map, figure.vertices_ll).map((pt) => {
      return L.point(cPt.x + (pt.x - cPt.x) * factor, cPt.y + (pt.y - cPt.y) * factor);
    });
    figure.vertices_ll = fromLayerPts(map, pts);
  }

  /** Move a single vertex (line/rect/polygon). */
  function setVertex(figure, index, latlng) {
    if (!figure.vertices_ll || index < 0 || index >= figure.vertices_ll.length) return;
    figure.vertices_ll[index] = [latlng[0], latlng[1]];
  }

  function updateLayerGeometry(layer, figure) {
    if (figure.figure_type === 'circle') {
      layer.setLatLng(figure.center_ll);
      layer.setRadius(figure.radius_m);
    } else if (figure.figure_type === 'line') {
      layer.setLatLngs(figure.vertices_ll);
    } else {
      layer.setLatLngs([figure.vertices_ll]);
    }
  }

  /** Bounding box center + top for rotate handle placement. */
  function boundsInfo(map, figure) {
    if (figure.figure_type === 'circle') {
      const c = L.latLng(figure.center_ll[0], figure.center_ll[1]);
      const cPt = map.latLngToLayerPoint(c);
      const rimLl = destinationPoint(figure.center_ll, 90, figure.radius_m);
      const rimPt = map.latLngToLayerPoint(L.latLng(rimLl[0], rimLl[1]));
      return {
        center: cPt,
        top: L.point(cPt.x, Math.min(cPt.y, rimPt.y)),
        rim: rimPt,
      };
    }
    const pts = toLayerPts(map, figure.vertices_ll);
    let minX = Infinity;
    let minY = Infinity;
    let maxX = -Infinity;
    let maxY = -Infinity;
    pts.forEach((p) => {
      minX = Math.min(minX, p.x);
      minY = Math.min(minY, p.y);
      maxX = Math.max(maxX, p.x);
      maxY = Math.max(maxY, p.y);
    });
    const center = L.point((minX + maxX) / 2, (minY + maxY) / 2);
    return {
      center,
      top: L.point(center.x, minY),
      vertices: pts,
    };
  }

  /** Destination from [lat,lon] bearing deg, distance m (approx). */
  function destinationPoint(ll, bearingDeg, distM) {
    const R = 6378137;
    const br = (bearingDeg * Math.PI) / 180;
    const lat1 = (ll[0] * Math.PI) / 180;
    const lon1 = (ll[1] * Math.PI) / 180;
    const ang = distM / R;
    const lat2 = Math.asin(
      Math.sin(lat1) * Math.cos(ang) + Math.cos(lat1) * Math.sin(ang) * Math.cos(br)
    );
    const lon2 =
      lon1 +
      Math.atan2(
        Math.sin(br) * Math.sin(ang) * Math.cos(lat1),
        Math.cos(ang) - Math.sin(lat1) * Math.sin(lat2)
      );
    return [(lat2 * 180) / Math.PI, (lon2 * 180) / Math.PI];
  }

  return {
    createLayer,
    applyStyle,
    toWgs84Payload,
    rectCorners,
    haversineM,
    fromApiFigure,
    centroid,
    translateFigure,
    rotateFigure,
    scaleFigure,
    setVertex,
    updateLayerGeometry,
    boundsInfo,
    destinationPoint,
    STYLE,
    LINE_STYLE,
    SELECT_STYLE,
    SELECT_LINE,
  };
})();
