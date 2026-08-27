/**
 * Nav2 keepout_filter inflation preview (GUI only, not saved to DB).
 * Cyan = inflation_radius, Orange = inscribed_radius.
 */
const InflationPreview = (() => {
  const CYAN = { color: '#00b4dc', fillColor: '#00c8ff', fillOpacity: 0.18, weight: 1 };
  const ORANGE = { color: '#e67800', fillColor: '#ffa000', fillOpacity: 0.22, weight: 1 };

  function metersToPixels(map, meters, atLatLng) {
    if (!meters || meters < 1e-6) return 0;
    const p0 = map.latLngToLayerPoint(atLatLng);
    const p1 = map.latLngToLayerPoint(
      L.latLng(atLatLng.lat + meters / 111320.0, atLatLng.lng)
    );
    return Math.max(0, Math.abs(p1.y - p0.y));
  }

  function refLatLng(figure) {
    if (figure.figure_type === 'circle' && figure.center_ll) {
      return L.latLng(figure.center_ll[0], figure.center_ll[1]);
    }
    const pts = figure.vertices_ll || [];
    if (!pts.length) return null;
    let lat = 0;
    let lon = 0;
    pts.forEach((p) => {
      lat += p[0];
      lon += p[1];
    });
    return L.latLng(lat / pts.length, lon / pts.length);
  }

  function ringOpts(style) {
    return {
      ...style,
      pane: 'inflationPane',
      interactive: false,
      bubblingMouseEvents: false,
    };
  }

  function strokeOpts(style, weightPx) {
    return {
      color: style.color,
      weight: Math.max(1, weightPx),
      opacity: 0.85,
      fill: false,
      lineJoin: 'round',
      lineCap: 'round',
      pane: 'inflationPane',
      interactive: false,
      bubblingMouseEvents: false,
    };
  }

  /**
   * Build preview layers for one figure (drawn under keepout geometry).
   * Returns array of Leaflet layers.
   */
  function layersForFigure(map, figure, inscribedM, inflationM) {
    const out = [];
    const ref = refLatLng(figure);
    if (!ref) return out;
    const inf = Math.max(0, inflationM);
    const ins = Math.max(0, inscribedM);

    if (figure.figure_type === 'circle') {
      const r = figure.radius_m || 0;
      if (inf > 0.01) {
        out.push(
          L.circle(figure.center_ll, {
            ...ringOpts(CYAN),
            radius: r + inf,
          })
        );
      }
      if (ins > 0.01) {
        out.push(
          L.circle(figure.center_ll, {
            ...ringOpts(ORANGE),
            radius: r + ins,
          })
        );
      }
      return out;
    }

    const verts = figure.vertices_ll;
    if (!verts || verts.length < 2) return out;

    const closed = figure.figure_type !== 'line';
    const path = closed ? [...verts, verts[0]] : verts;

    // Outer inflation first, then inscribed (same order as Qt paintInflationPreview)
    const infPx = metersToPixels(map, 2 * inf, ref);
    const insPx = metersToPixels(map, 2 * ins, ref);
    if (infPx >= 1) {
      out.push(L.polyline(path, strokeOpts(CYAN, infPx)));
    }
    if (insPx >= 1) {
      out.push(L.polyline(path, strokeOpts(ORANGE, insPx)));
    }
    return out;
  }

  return { layersForFigure, metersToPixels };
})();
