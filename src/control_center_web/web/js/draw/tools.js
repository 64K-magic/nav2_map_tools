/**
 * Drawing tools + select-mode edit (move / resize / rotate).
 */
class DrawController {
  constructor(map, onChange) {
    this.map = map;
    this.onChange = onChange || (() => {});
    this.tool = 'select';
    this.figures = []; // {figure, layer}
    this.selectedIndex = -1;
    this._draft = null;
    this._preview = null;
    this._polyPts = [];
    this._bound = false;
    this._handles = L.layerGroup().addTo(map);
    this._edit = null; // active drag state
    this._container = map.getContainer();
    // Inflation preview under keepout geometry (GUI only)
    if (!map.getPane('inflationPane')) {
      map.createPane('inflationPane');
      map.getPane('inflationPane').style.zIndex = 350; // below overlayPane 400
    }
    this._inflation = L.layerGroup([], { pane: 'inflationPane' }).addTo(map);
    this._inflationOpts = {
      show: true,
      inscribed_m: 0.4,
      inflation_m: 1.0,
    };
  }

  setInflationPreview(show, inscribedM, inflationM) {
    this._inflationOpts = {
      show: !!show,
      inscribed_m: Math.max(0, Number(inscribedM) || 0),
      inflation_m: Math.max(0, Number(inflationM) || 0),
    };
    this.refreshInflationPreview();
  }

  refreshInflationPreview() {
    this._inflation.clearLayers();
    const opt = this._inflationOpts;
    if (!opt || !opt.show) return;
    this.figures.forEach((item) => {
      InflationPreview.layersForFigure(
        this.map,
        item.figure,
        opt.inscribed_m,
        opt.inflation_m
      ).forEach((ly) => this._inflation.addLayer(ly));
    });
  }

  setTool(tool) {
    this.tool = tool;
    this._cancelDraft();
    if (tool === 'select') {
      this.map.dragging.enable();
      this.map.doubleClickZoom.enable();
      this._setCursor('grab');
    } else {
      this.map.dragging.disable();
      this.map.doubleClickZoom.disable();
      this._setCursor('crosshair');
      this.selectFigure(-1);
    }
  }

  _setCursor(kind) {
    const el = this._container;
    el.classList.remove(
      'cursor-select',
      'cursor-draw',
      'cursor-move',
      'cursor-resize',
      'cursor-rotate'
    );
    if (kind === 'grab' || kind === 'select') el.classList.add('cursor-select');
    else if (kind === 'crosshair' || kind === 'draw') el.classList.add('cursor-draw');
    else if (kind === 'move') el.classList.add('cursor-move');
    else if (kind === 'resize') el.classList.add('cursor-resize');
    else if (kind === 'rotate') el.classList.add('cursor-rotate');
  }

  bind() {
    if (this._bound) return;
    this._bound = true;
    this.map.on('click', (e) => this._onClick(e));
    this.map.on('mousemove', (e) => this._onMove(e));
    this.map.on('mousedown', (e) => this._onMouseDown(e));
    this.map.on('mouseup', (e) => this._onMouseUp(e));
    this.map.on('dblclick', (e) => this._onDblClick(e));
    this.map.on('contextmenu', (e) => {
      L.DomEvent.preventDefault(e);
      if (this.tool === 'polygon') this._finishPolygon();
    });
    // Keep handles / inflation aligned when zooming/panning
    this.map.on('zoomend moveend', () => {
      if (this.selectedIndex >= 0) this._rebuildHandles();
      this.refreshInflationPreview();
    });
    document.addEventListener('keydown', (ev) => {
      if (ev.key === 'Enter' && this.tool === 'polygon') this._finishPolygon();
      if (ev.key === 'Escape') {
        this._cancelDraft();
        this._endEdit();
        if (this.tool === 'select') this.selectFigure(-1);
      }
      if ((ev.key === 'Delete' || ev.key === 'Backspace') && this.tool === 'select') {
        if (this.selectedIndex >= 0 && !ev.target.matches('input,textarea')) {
          ev.preventDefault();
          this.deleteSelected();
        }
      }
    });
    document.addEventListener('mouseup', () => {
      if (this._edit) this._endEdit();
    });
    this._setCursor('grab');
  }

  clear() {
    this.selectFigure(-1);
    this.figures.forEach((f) => this.map.removeLayer(f.layer));
    this.figures = [];
    this._cancelDraft();
    this.refreshInflationPreview();
    this.onChange();
  }

  undo() {
    if (this.tool === 'polygon' && this._polyPts.length) {
      this._polyPts.pop();
      this._refreshPolyPreview();
      return;
    }
    if (this.selectedIndex === this.figures.length - 1) this.selectFigure(-1);
    const last = this.figures.pop();
    if (last) {
      this.map.removeLayer(last.layer);
      this.refreshInflationPreview();
      this.onChange();
    }
  }

  deleteSelected() {
    const i = this.selectedIndex;
    if (i < 0) return;
    const item = this.figures[i];
    this.map.removeLayer(item.layer);
    this.figures.splice(i, 1);
    this.selectFigure(-1);
    // rebind select handlers with new indices
    this.figures.forEach((f, idx) => this._attachSelect(f.layer, idx));
    this.refreshInflationPreview();
    this.onChange();
  }

  loadFigures(list) {
    this.selectFigure(-1);
    this.figures.forEach((f) => this.map.removeLayer(f.layer));
    this.figures = [];
    this._cancelDraft();
    list.forEach((raw) => {
      const fig = KeepoutShapes.fromApiFigure(raw);
      const layer = KeepoutShapes.createLayer(fig);
      layer.addTo(this.map);
      this._attachSelect(layer, this.figures.length);
      this.figures.push({ figure: fig, layer });
    });
    this.refreshInflationPreview();
    this.onChange();
  }

  toWgs84List() {
    return this.figures.map((f, i) => {
      const payload = KeepoutShapes.toWgs84Payload(f.figure);
      if (!payload.figure_name) payload.figure_name = `fig_${i + 1}`;
      return payload;
    });
  }

  toMapList(mapName) {
    return this.figures.map((f, i) => {
      const payload = MapCoords.figureToMapApi(f.figure, mapName);
      if (!payload.figure_name) payload.figure_name = `fig_${i + 1}`;
      return payload;
    });
  }

  selectFigure(index) {
    this.selectedIndex = index;
    this.figures.forEach((f, i) => {
      KeepoutShapes.applyStyle(f.layer, f.figure, i === index);
    });
    this._rebuildHandles();
    this.onChange(index >= 0 ? index : undefined);
  }

  _attachSelect(layer, index) {
    layer.off('click');
    layer.off('mousedown');
    layer.on('click', (e) => {
      if (this.tool !== 'select') return;
      L.DomEvent.stopPropagation(e);
      this.selectFigure(index);
    });
    layer.on('mousedown', (e) => {
      if (this.tool !== 'select') return;
      L.DomEvent.stopPropagation(e);
      L.DomEvent.preventDefault(e);
      if (this.selectedIndex !== index) this.selectFigure(index);
      this._beginMove(e);
    });
  }

  _cancelDraft() {
    if (this._preview) {
      this.map.removeLayer(this._preview);
      this._preview = null;
    }
    this._draft = null;
    this._polyPts = [];
  }

  _onClick(e) {
    if (this.tool === 'select') {
      if (this._edit || this._suppressDeselect) return;
      this.selectFigure(-1);
      return;
    }
    const ll = [e.latlng.lat, e.latlng.lng];

    if (this.tool === 'polygon') {
      this._polyPts.push(ll);
      this._refreshPolyPreview();
      return;
    }

    if (!this._draft) {
      this._draft = { start: ll };
      return;
    }
    this._finishTwoPoint(ll);
  }

  _onMove(e) {
    if (this._edit) {
      this._continueEdit(e);
      return;
    }
    if (!this._draft && this.tool !== 'polygon') return;
    const ll = [e.latlng.lat, e.latlng.lng];

    if (this.tool === 'polygon' && this._polyPts.length) {
      this._refreshPolyPreview(ll);
      return;
    }
    if (!this._draft) return;

    if (this._preview) this.map.removeLayer(this._preview);
    const a = this._draft.start;
    if (this.tool === 'line') {
      this._preview = L.polyline([a, ll], {
        ...KeepoutShapes.LINE_STYLE,
        dashArray: '6 4',
        color: '#ffaa00',
      }).addTo(this.map);
    } else if (this.tool === 'rectangle') {
      this._preview = L.polygon(KeepoutShapes.rectCorners(a, ll), {
        ...KeepoutShapes.STYLE,
        color: '#ffaa00',
        dashArray: '6 4',
      }).addTo(this.map);
    } else if (this.tool === 'circle') {
      const r = MapCoords.distanceM(a, ll);
      const radius =
        typeof MapCoords !== 'undefined' && MapCoords.isPgm()
          ? MapCoords.circleDisplayRadius(Math.max(r, 0.5))
          : Math.max(r, 0.5);
      this._preview = L.circle(a, {
        ...KeepoutShapes.STYLE,
        color: '#ffaa00',
        dashArray: '6 4',
        radius,
      }).addTo(this.map);
    }
  }

  _onMouseDown() {
    /* move started via layer/handle handlers */
  }

  _onMouseUp() {
    if (this._edit) this._endEdit();
  }

  _onDblClick(e) {
    if (this.tool === 'polygon') {
      L.DomEvent.preventDefault(e);
      this._finishPolygon();
    }
  }

  _refreshPolyPreview(cursor) {
    if (this._preview) this.map.removeLayer(this._preview);
    const pts = cursor ? [...this._polyPts, cursor] : [...this._polyPts];
    if (pts.length < 2) {
      this._preview = null;
      return;
    }
    this._preview = L.polygon(pts, {
      ...KeepoutShapes.STYLE,
      color: '#ffaa00',
      dashArray: '6 4',
    }).addTo(this.map);
  }

  _finishPolygon() {
    if (this._polyPts.length < 3) {
      this._cancelDraft();
      return;
    }
    this._commit({
      figure_type: 'polygon',
      figure_name: '',
      vertices_ll: this._polyPts.slice(),
    });
    this._cancelDraft();
  }

  _finishTwoPoint(end) {
    const start = this._draft.start;
    let fig = null;
    if (this.tool === 'line') {
      fig = { figure_type: 'line', figure_name: '', vertices_ll: [start, end] };
    } else if (this.tool === 'rectangle') {
      fig = {
        figure_type: 'rectangle',
        figure_name: '',
        vertices_ll: KeepoutShapes.rectCorners(start, end),
      };
    } else if (this.tool === 'circle') {
      fig = {
        figure_type: 'circle',
        figure_name: '',
        center_ll: start,
        radius_m: Math.max(MapCoords.distanceM(start, end), 0.5),
      };
    }
    if (fig) this._commit(fig);
    this._cancelDraft();
  }

  _commit(fig) {
    const layer = KeepoutShapes.createLayer(fig);
    layer.addTo(this.map);
    const idx = this.figures.length;
    this._attachSelect(layer, idx);
    this.figures.push({ figure: fig, layer });
    this.refreshInflationPreview();
    this.onChange();
  }

  /* ---------- edit: move / resize / rotate ---------- */

  _rebuildHandles() {
    this._handles.clearLayers();
    if (this.selectedIndex < 0 || this.tool !== 'select') return;
    const item = this.figures[this.selectedIndex];
    if (!item) return;
    const fig = item.figure;
    const info = KeepoutShapes.boundsInfo(this.map, fig);

    if (fig.figure_type === 'circle') {
      this._addHandle(info.rim, 'resize', { kind: 'radius' });
    } else {
      fig.vertices_ll.forEach((ll, vi) => {
        const pt = this.map.latLngToLayerPoint(L.latLng(ll[0], ll[1]));
        this._addHandle(pt, 'resize', { kind: 'vertex', index: vi });
      });
    }

    // rotate handle at geometric center (line → segment mid / vertex centroid)
    if (fig.figure_type !== 'circle') {
      const c = KeepoutShapes.centroid(fig);
      const rotPt = this.map.latLngToLayerPoint(L.latLng(c[0], c[1]));
      this._addHandle(rotPt, 'rotate', { kind: 'rotate' });
    }
  }

  _addHandle(layerPt, cursorKind, meta) {
    const ll = this.map.layerPointToLatLng(layerPt);
    const icon = L.divIcon({
      className: cursorKind === 'rotate' ? 'edit-handle edit-handle-rotate' : 'edit-handle',
      iconSize: [12, 12],
      iconAnchor: [6, 6],
    });
    const marker = L.marker(ll, {
      icon,
      draggable: false,
      interactive: true,
      zIndexOffset: cursorKind === 'rotate' ? 2000 : 1000,
    });
    marker.on('mousedown', (e) => {
      L.DomEvent.stopPropagation(e);
      L.DomEvent.preventDefault(e);
      this._beginHandleDrag(e, meta);
    });
    this._handles.addLayer(marker);
  }

  _beginMove(e) {
    if (this.selectedIndex < 0) return;
    this.map.dragging.disable();
    this._setCursor('move');
    this._edit = {
      mode: 'move',
      last: [e.latlng.lat, e.latlng.lng],
    };
    this.map.on('mousemove', this._onEditMoveBound);
    this.map.on('mouseup', this._onEditUpBound);
  }

  _beginHandleDrag(e, meta) {
    if (this.selectedIndex < 0) return;
    this.map.dragging.disable();
    const item = this.figures[this.selectedIndex];
    const fig = item.figure;
    const c = KeepoutShapes.centroid(fig);
    const cPt = this.map.latLngToLayerPoint(L.latLng(c[0], c[1]));
    const curPt = this.map.latLngToLayerPoint(e.latlng);

    if (meta.kind === 'rotate') {
      this._setCursor('rotate');
      this._edit = {
        mode: 'rotate',
        armed: false,
        lastAngle: 0,
      };
    } else if (meta.kind === 'radius') {
      this._setCursor('resize');
      this._edit = { mode: 'radius' };
    } else if (meta.kind === 'vertex') {
      this._setCursor('resize');
      this._edit = { mode: 'vertex', index: meta.index };
    } else {
      // scale via corner distance
      this._setCursor('resize');
      const dist0 = Math.hypot(curPt.x - cPt.x, curPt.y - cPt.y) || 1;
      this._edit = { mode: 'scale', dist0 };
    }

    this.map.on('mousemove', this._onEditMoveBound);
    this.map.on('mouseup', this._onEditUpBound);
  }

  get _onEditMoveBound() {
    if (!this.__onEditMoveBound) {
      this.__onEditMoveBound = (e) => this._continueEdit(e);
    }
    return this.__onEditMoveBound;
  }

  get _onEditUpBound() {
    if (!this.__onEditUpBound) {
      this.__onEditUpBound = () => this._endEdit();
    }
    return this.__onEditUpBound;
  }

  _continueEdit(e) {
    if (!this._edit || this.selectedIndex < 0) return;
    const item = this.figures[this.selectedIndex];
    const fig = item.figure;
    const ll = [e.latlng.lat, e.latlng.lng];

    if (this._edit.mode === 'move') {
      const dLat = ll[0] - this._edit.last[0];
      const dLon = ll[1] - this._edit.last[1];
      KeepoutShapes.translateFigure(fig, dLat, dLon);
      this._edit.last = ll;
    } else if (this._edit.mode === 'vertex') {
      KeepoutShapes.setVertex(fig, this._edit.index, ll);
    } else if (this._edit.mode === 'radius') {
      fig.radius_m = Math.max(0.5, MapCoords.distanceM(fig.center_ll, ll));
    } else if (this._edit.mode === 'rotate') {
      const c = KeepoutShapes.centroid(fig);
      const cPt = this.map.latLngToLayerPoint(L.latLng(c[0], c[1]));
      const curPt = this.map.latLngToLayerPoint(e.latlng);
      // Ignore until cursor leaves the center handle (angle undefined at pivot).
      if (Math.hypot(curPt.x - cPt.x, curPt.y - cPt.y) < 6) return;
      const ang = Math.atan2(curPt.y - cPt.y, curPt.x - cPt.x);
      if (!this._edit.armed) {
        this._edit.lastAngle = ang;
        this._edit.armed = true;
        return;
      }
      const delta = ang - this._edit.lastAngle;
      KeepoutShapes.rotateFigure(this.map, fig, delta);
      this._edit.lastAngle = ang;
    } else if (this._edit.mode === 'scale') {
      const c = KeepoutShapes.centroid(fig);
      const cPt = this.map.latLngToLayerPoint(L.latLng(c[0], c[1]));
      const curPt = this.map.latLngToLayerPoint(e.latlng);
      const dist = Math.hypot(curPt.x - cPt.x, curPt.y - cPt.y) || 1;
      const factor = dist / this._edit.dist0;
      KeepoutShapes.scaleFigure(this.map, fig, factor);
      this._edit.dist0 = dist;
    }

    KeepoutShapes.updateLayerGeometry(item.layer, fig);
    this._rebuildHandles();
    this.refreshInflationPreview();
  }

  _endEdit() {
    if (!this._edit) return;
    this._edit = null;
    this._suppressDeselect = true;
    setTimeout(() => {
      this._suppressDeselect = false;
    }, 50);
    this.map.off('mousemove', this._onEditMoveBound);
    this.map.off('mouseup', this._onEditUpBound);
    if (this.tool === 'select') {
      this.map.dragging.enable();
      this._setCursor('grab');
    } else {
      this._setCursor('crosshair');
    }
    this._rebuildHandles();
    this.refreshInflationPreview();
    this.onChange(this.selectedIndex);
  }
}
