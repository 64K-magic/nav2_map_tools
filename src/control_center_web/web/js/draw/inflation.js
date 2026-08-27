/**
 * Nav2 keepout_filter inflation preview (GUI only, not saved to DB).
 * Cyan = inflation_radius, Orange = inscribed_radius.
 * Bands extend outward from the keepout boundary only (not inward).
 */
const InflationPreview = (() => {
  const CYAN = { color: '#00b4dc', fillColor: '#00c8ff', fillOpacity: 0.38, weight: 1 };
  const ORANGE = { color: '#e67800', fillColor: '#ffa000', fillOpacity: 0.62, weight: 1 };

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
      stroke: false,
    };
  }

  function toLayerPts(map, latlngs) {
    return latlngs.map(([lat, lon]) => map.latLngToLayerPoint(L.latLng(lat, lon)));
  }

  function fromLayerPts(map, pts) {
    return pts.map((p) => {
      const ll = map.layerPointToLatLng(p);
      return [ll.lat, ll.lng];
    });
  }

  function addPt(a, b) {
    return L.point(a.x + b.x, a.y + b.y);
  }

  function subPt(a, b) {
    return L.point(a.x - b.x, a.y - b.y);
  }

  function scalePt(p, s) {
    return L.point(p.x * s, p.y * s);
  }

  function lenPt(p) {
    return Math.hypot(p.x, p.y);
  }

  function normPt(p) {
    const l = lenPt(p);
    return l > 1e-9 ? scalePt(p, 1 / l) : L.point(0, 0);
  }

  function leftNormal(v) {
    return L.point(-v.y, v.x);
  }

  function lineIntersect(p1, p2, p3, p4) {
    const d1 = subPt(p2, p1);
    const d2 = subPt(p4, p3);
    const denom = d1.x * d2.y - d1.y * d2.x;
    if (Math.abs(denom) < 1e-9) return null;
    const t = ((p3.x - p1.x) * d2.y - (p3.y - p1.y) * d2.x) / denom;
    return addPt(p1, scalePt(d1, t));
  }

  /** Outward parallel offset for a closed polygon in layer pixels. */
  function offsetClosedOutward(layerPts, distPx) {
    const n = layerPts.length;
    if (n < 3 || distPx < 0.5) return null;

    let cx = 0;
    let cy = 0;
    layerPts.forEach((p) => {
      cx += p.x;
      cy += p.y;
    });
    cx /= n;
    cy /= n;
    const centroid = L.point(cx, cy);

    const out = [];
    for (let i = 0; i < n; i += 1) {
      const prev = layerPts[(i - 1 + n) % n];
      const curr = layerPts[i];
      const next = layerPts[(i + 1) % n];

      const e1 = normPt(subPt(curr, prev));
      const e2 = normPt(subPt(next, curr));

      let n1 = leftNormal(e1);
      let n2 = leftNormal(e2);
      const toCenter = subPt(centroid, curr);
      if (n1.x * toCenter.x + n1.y * toCenter.y > 0) {
        n1 = scalePt(n1, -1);
      }
      if (n2.x * toCenter.x + n2.y * toCenter.y > 0) {
        n2 = scalePt(n2, -1);
      }

      const p1 = addPt(curr, scalePt(n1, distPx));
      const p2 = addPt(curr, scalePt(n2, distPx));
      const corner =
        lineIntersect(p1, addPt(p1, e1), p2, addPt(p2, e2)) ||
        addPt(curr, scalePt(normPt(addPt(n1, n2)), distPx));
      out.push(corner);
    }
    return out;
  }

  /** Symmetric corridor around an open polyline (both sides). */
  function offsetOpenCorridor(layerPts, halfWidthPx) {
    if (layerPts.length < 2 || halfWidthPx < 0.5) return null;

    const left = [];
    const right = [];
    for (let i = 0; i < layerPts.length; i += 1) {
      const prev = layerPts[Math.max(0, i - 1)];
      const curr = layerPts[i];
      const next = layerPts[Math.min(layerPts.length - 1, i + 1)];

      let tangent = normPt(subPt(next, prev));
      if (lenPt(tangent) < 1e-9) {
        tangent = normPt(subPt(next, curr));
      }
      const normal = leftNormal(tangent);
      left.push(addPt(curr, scalePt(normal, halfWidthPx)));
      right.push(addPt(curr, scalePt(normal, -halfWidthPx)));
    }
    return left.concat([...right].reverse());
  }

  function circleRingLatLngs(centerLl, rInnerM, rOuterM, segments = 64) {
    const outer = [];
    const inner = [];
    for (let i = 0; i <= segments; i += 1) {
      const a = (2 * Math.PI * i) / segments;
      const br = (a * 180) / Math.PI;
      outer.push(KeepoutShapes.destinationPoint(centerLl, br, Math.max(0, rOuterM)));
      inner.push(KeepoutShapes.destinationPoint(centerLl, br, Math.max(0, rInnerM)));
    }
    return { outer, inner };
  }

  function ringPolygon(innerLatLngs, outerLatLngs, style) {
    const outer = outerLatLngs.slice();
    const hole = [...innerLatLngs].reverse();
    if (outer.length < 3 || hole.length < 3) return null;
    return L.polygon([outer, hole], ringOpts(style));
  }

  function outwardBandLayer(map, innerLatLngs, distM, ref, style) {
    const distPx = metersToPixels(map, distM, ref);
    if (distPx < 0.5) return null;

    const innerPts = toLayerPts(map, innerLatLngs);
    const outerPts = offsetClosedOutward(innerPts, distPx);
    if (!outerPts) return null;

    return ringPolygon(innerLatLngs, fromLayerPts(map, outerPts), style);
  }

  function lineBandLayer(map, pathLatLngs, halfDistM, ref, style) {
    const halfPx = metersToPixels(map, halfDistM, ref);
    if (halfPx < 0.5) return null;

    const corridor = offsetOpenCorridor(toLayerPts(map, pathLatLngs), halfPx);
    if (!corridor || corridor.length < 3) return null;
    return L.polygon(fromLayerPts(map, corridor), ringOpts(style));
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
        const { outer, inner } = circleRingLatLngs(figure.center_ll, r, r + inf);
        const ly = ringPolygon(inner, outer, CYAN);
        if (ly) out.push(ly);
      }
      if (ins > 0.01) {
        const { outer, inner } = circleRingLatLngs(figure.center_ll, r, r + ins);
        const ly = ringPolygon(inner, outer, ORANGE);
        if (ly) out.push(ly);
      }
      return out;
    }

    const verts = figure.vertices_ll;
    if (!verts || verts.length < 2) return out;

    const closed = figure.figure_type !== 'line';
    const path = closed ? verts : verts;

    if (closed) {
      // Outer inflation first, then inscribed (same order as Qt paintInflationPreview)
      if (inf > 0.01) {
        const ly = outwardBandLayer(map, path, inf, ref, CYAN);
        if (ly) out.push(ly);
      }
      if (ins > 0.01) {
        const ly = outwardBandLayer(map, path, ins, ref, ORANGE);
        if (ly) out.push(ly);
      }
      return out;
    }

    if (inf > 0.01) {
      const ly = lineBandLayer(map, path, inf, ref, CYAN);
      if (ly) out.push(ly);
    }
    if (ins > 0.01) {
      const ly = lineBandLayer(map, path, ins, ref, ORANGE);
      if (ly) out.push(ly);
    }
    return out;
  }

  return { layersForFigure, metersToPixels };
})();
