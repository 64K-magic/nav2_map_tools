/**
 * Keepout Edit frontend — tile map or PGM map + draw tools + API save/load.
 */
(function () {
  const $ = (id) => document.getElementById(id);
  const statusEl = $('status');
  const figureList = $('figureList');

  let mapMode = 'tile';
  let pgmMeta = null;
  let map;
  let tileLayer;
  let pgmLayer;
  let drawer;
  let gpsMarker;
  let robotTracker;
  let appCfg = {};

  function setStatus(msg) {
    statusEl.textContent = msg;
  }

  function origin() {
    return {
      lat: parseFloat($('originLat').value),
      lon: parseFloat($('originLon').value),
      yaw_deg: parseFloat($('originYaw').value) || 0,
    };
  }

  function getMapMode() {
    const sel = $('mapModeSelect');
    return sel ? sel.value : mapMode;
  }

  function isTileMode() {
    return getMapMode() === 'tile';
  }

  function requireMapName() {
    if (getMapMode() === 'pgm' && pgmMeta && pgmMeta.name) {
      return pgmMeta.name;
    }
    const custom = ($('mapNameCustom').value || '').trim();
    if (custom) return custom;
    const selected = ($('mapNameSelect').value || '').trim();
    if (!selected) throw new Error('请选择 MapName，或填写新地图名');
    return selected;
  }

  async function refreshMapNameOptions(prefer) {
    const sel = $('mapNameSelect');
    const prev = prefer || sel.value || ($('mapNameCustom').value || '').trim();
    sel.innerHTML = '<option value="">— 从数据库选择 —</option>';
    try {
      const names = await KeepoutApi.listMaps();
      (names || []).forEach((name) => {
        const opt = document.createElement('option');
        opt.value = name;
        opt.textContent = name;
        sel.appendChild(opt);
      });
      if (prev) {
        const hit = Array.from(sel.options).some((o) => o.value === prev);
        if (hit) {
          sel.value = prev;
          $('mapNameCustom').value = '';
        } else {
          $('mapNameCustom').value = prev;
          sel.value = '';
        }
      } else if (names && names.includes('map1')) {
        sel.value = 'map1';
      } else if (names && names.length === 1) {
        sel.value = names[0];
      }
      setStatus(
        names && names.length
          ? `地图列表 ${names.length} 个：${names.join(', ')}`
          : '数据库尚无 map_name，可在「新地图名」填写后保存'
      );
      return names || [];
    } catch (e) {
      setStatus('加载地图列表失败: ' + e.message);
      return [];
    }
  }

  async function refreshPgmMapOptions(prefer) {
    const sel = $('pgmMapSelect');
    if (!sel) return [];
    sel.innerHTML = '<option value="">— 选择 PGM 地图 —</option>';
    try {
      const maps = await KeepoutApi.listPgmMaps();
      (maps || []).forEach((m) => {
        const opt = document.createElement('option');
        opt.value = m.name;
        opt.textContent = `${m.name} (${m.width}×${m.height})`;
        sel.appendChild(opt);
      });
      if (prefer) sel.value = prefer;
      return maps || [];
    } catch (e) {
      setStatus('PGM 列表失败: ' + e.message);
      return [];
    }
  }

  function tileUrlTemplate(url) {
    return url.replace(/\{level\}/g, '{z}');
  }

  function fillPgmCoordPanel(meta) {
    const set = (id, val) => {
      const el = $(id);
      if (el) el.value = val != null && val !== '' ? String(val) : '';
    };
    if (!meta) {
      ['pgmResolution', 'pgmOriginX', 'pgmOriginY', 'pgmOriginYaw', 'pgmSize'].forEach(
        (id) => set(id, '')
      );
      return;
    }
    const origin = meta.origin || [];
    set('pgmResolution', meta.resolution);
    set('pgmOriginX', meta.origin_x != null ? meta.origin_x : origin[0]);
    set('pgmOriginY', meta.origin_y != null ? meta.origin_y : origin[1]);
    set('pgmOriginYaw', meta.origin_yaw != null ? meta.origin_yaw : origin[2]);
    set('pgmSize', `${meta.width} × ${meta.height} px`);
  }

  function updateModePanels() {
    const tile = isTileMode();
    $('originPanel').style.display = tile ? '' : 'none';
    $('pgmCoordPanel').style.display = tile ? 'none' : '';
    $('pgmPanel').style.display = tile ? 'none' : '';
    if ($('tilePanel')) $('tilePanel').style.display = '';
    $('brandTitle').textContent = tile ? '瓦片图禁行区' : 'PGM 地图禁行区';
    if (!tile && !pgmMeta) {
      fillPgmCoordPanel(null);
    }
  }

  function destroyMapLayers() {
    if (robotTracker) {
      robotTracker.stop();
      robotTracker = null;
    }
    if (drawer) {
      drawer.clear();
      drawer = null;
    }
    if (map) {
      map.remove();
      map = null;
    }
    tileLayer = null;
    pgmLayer = null;
    gpsMarker = null;
  }

  function attachDrawer() {
    drawer = new DrawController(map, refreshFigureList);
    drawer.bind();
    applyInflationFromUi();
    refreshFigureList();
  }

  async function initRobotTracking(cfg, health) {
    robotTracker = new RobotTracker(map, origin, getMapMode);
    const robotCfg = (cfg && cfg.robot) || {};
    if (robotCfg.track_poll_hz) {
      robotTracker.setPollHz(robotCfg.track_poll_hz);
    }

    const trackChk = $('trackRobotChk');
    const followChk = $('followRobotChk');

    trackChk.onchange = () => {
      if (trackChk.checked) robotTracker.start();
      else robotTracker.stop();
    };
    followChk.onchange = () => {
      robotTracker.setFollow(followChk.checked);
    };

    if (health && health.ros_enabled && trackChk.checked) {
      robotTracker.start();
    }
  }

  function initTileMap(cfg) {
    destroyMapLayers();
    mapMode = 'tile';
    pgmMeta = null;
    MapCoords.setTileMode();
    fillPgmCoordPanel(null);
    updateModePanels();

    const originCfg = cfg.default_origin || {};
    const tileCfg = cfg.tile || {};
    const defaultTileUrl =
      tileCfg.default_tile_url ||
      'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{level}/{y}/{x}';
    const center = [
      tileCfg.default_center_lat != null ? tileCfg.default_center_lat : originCfg.lat || 38.161479,
      tileCfg.default_center_lon != null ? tileCfg.default_center_lon : originCfg.lon || -122.454630,
    ];
    const zoom = tileCfg.default_zoom != null ? tileCfg.default_zoom : 18;

    map = L.map('map', {
      center,
      zoom,
      zoomControl: true,
      attributionControl: false,
      doubleClickZoom: true,
    });
    setTileLayer(defaultTileUrl);
    attachDrawer();
  }

  function setTileLayer(url) {
    if (tileLayer) map.removeLayer(tileLayer);
    tileLayer = L.tileLayer(tileUrlTemplate(url), {
      maxZoom: 22,
      crossOrigin: true,
    }).addTo(map);
  }

  async function loadPgmMap(name) {
    if (!name) throw new Error('请选择 PGM 地图');
    const meta = await KeepoutApi.getPgmMap(name);
    destroyMapLayers();
    mapMode = 'pgm';
    pgmMeta = meta;
    MapCoords.setPgmMode(meta);
    fillPgmCoordPanel(meta);
    updateModePanels();

    const h = meta.height;
    const w = meta.width;
    map = L.map('map', {
      crs: L.CRS.Simple,
      minZoom: -3,
      maxZoom: 5,
      zoomControl: true,
      attributionControl: false,
      doubleClickZoom: true,
    });
    // Keep PGM under inflation (350) / keepout (400) / robot (450) panes.
    if (!map.getPane('pgmPane')) {
      map.createPane('pgmPane');
      map.getPane('pgmPane').style.zIndex = 200;
    }
    const bounds = [
      [0, 0],
      [h, w],
    ];
    pgmLayer = L.imageOverlay(KeepoutApi.pgmImageUrl(name), bounds, {
      pane: 'pgmPane',
      interactive: false,
    }).addTo(map);
    map.fitBounds(bounds);
    map.setMaxBounds(
      L.latLngBounds(
        L.latLng(-h * 0.05, -w * 0.05),
        L.latLng(h * 1.05, w * 1.05)
      )
    );

    $('mapNameCustom').value = name;
    $('mapNameSelect').value = '';
    attachDrawer();

    const health = await KeepoutApi.health().catch(() => null);
    await initRobotTracking(appCfg, health);
    setStatus(`已加载 PGM: ${name} (${w}×${h}, res=${meta.resolution})`);
  }

  function applyInflationFromUi() {
    if (!drawer) return;
    drawer.setInflationPreview(
      $('showInflationChk').checked,
      parseFloat($('inscribedRadius').value),
      parseFloat($('inflationRadius').value)
    );
  }

  function refreshFigureList(selectedIdx) {
    figureList.innerHTML = '';
    if (!drawer) return;
    const n = drawer.figures.length;
    const summary = $('figureListSummary');
    if (summary) {
      summary.textContent = n ? `图形列表 (${n})` : '图形列表';
    }
    const sel =
      selectedIdx !== undefined && selectedIdx !== null
        ? selectedIdx
        : drawer.selectedIndex;
    drawer.figures.forEach((f, i) => {
      const li = document.createElement('li');
      const name = f.figure.figure_name || `fig_${i + 1}`;
      li.textContent = `${name} · ${f.figure.figure_type}`;
      if (i === sel) li.classList.add('selected');
      li.onclick = () => {
        document.querySelectorAll('.tool').forEach((b) => b.classList.remove('active'));
        const selBtn = document.querySelector('.tool[data-tool="select"]');
        if (selBtn) selBtn.classList.add('active');
        drawer.setTool('select');
        drawer.selectFigure(i);
      };
      figureList.appendChild(li);
    });
  }

  function wireUi() {
    document.querySelectorAll('.tool').forEach((btn) => {
      btn.addEventListener('click', () => {
        document.querySelectorAll('.tool').forEach((b) => b.classList.remove('active'));
        btn.classList.add('active');
        drawer.setTool(btn.dataset.tool);
        const tips = {
          select: '选择：拖动移动，顶点缩放，圆点旋转',
          line: '线：两点完成',
          rectangle: '矩形：两点对角',
          polygon: '多边形：左键加点，双击/Enter/右键结束',
          circle: '圆：中心 + 半径点',
        };
        setStatus(tips[btn.dataset.tool] || btn.dataset.tool);
      });
    });

    $('btnUndo').onclick = () => drawer && drawer.undo();
    $('btnClear').onclick = () => {
      if (drawer && confirm('清空当前所有图形？')) drawer.clear();
    };

    $('showInflationChk').addEventListener('change', applyInflationFromUi);
    $('inscribedRadius').addEventListener('change', applyInflationFromUi);
    $('inscribedRadius').addEventListener('input', applyInflationFromUi);
    $('inflationRadius').addEventListener('change', applyInflationFromUi);
    $('inflationRadius').addEventListener('input', applyInflationFromUi);

    $('mapModeSelect').addEventListener('change', async () => {
      const mode = $('mapModeSelect').value;
      if (mode === 'tile') {
        initTileMap(appCfg);
        const health = await KeepoutApi.health().catch(() => null);
        await initRobotTracking(appCfg, health);
        setStatus('已切换到瓦片地图模式');
        return;
      }
      mapMode = 'pgm';
      await refreshPgmMapOptions();
      updateModePanels();
      if (robotTracker) {
        robotTracker.stop();
        robotTracker.start();
      }
      setStatus('PGM 模式：选择地图后点击「加载 PGM」');
    });

    $('btnLoadPgm').onclick = async () => {
      try {
        const name = ($('pgmMapSelect').value || '').trim();
        await loadPgmMap(name);
        await refreshMapNameOptions(name);
      } catch (e) {
        setStatus('加载 PGM 失败: ' + e.message);
      }
    };

    $('btnRefreshPgm').onclick = () => refreshPgmMapOptions($('pgmMapSelect').value);

    $('btnGoMapCenter').onclick = () => {
      if (getMapMode() === 'pgm' && pgmMeta && map) {
        map.fitBounds([
          [0, 0],
          [pgmMeta.height, pgmMeta.width],
        ]);
      }
    };

    $('btnGoCenter').onclick = () => {
      if (getMapMode() === 'pgm' && pgmMeta) {
        $('btnGoMapCenter').click();
        return;
      }
      const o = origin();
      map.setView([o.lat, o.lon], map.getZoom());
    };

    $('btnRefreshMaps').onclick = () => refreshMapNameOptions();

    $('btnDeleteMap').onclick = async () => {
      const name = ($('mapNameSelect').value || '').trim() || ($('mapNameCustom').value || '').trim();
      if (!name) {
        setStatus('请先选择或填写地图名');
        return;
      }
      if (!confirm(`确定从数据库删除地图「${name}」下的全部禁行区？此操作不可恢复。`)) {
        return;
      }
      try {
        const r = await KeepoutApi.deleteMap(name, $('notifyOnSave').checked);
        drawer.clear();
        $('mapNameCustom').value = '';
        await refreshMapNameOptions();
        setStatus(r.message || `已删除 ${name}`);
      } catch (e) {
        setStatus('删除失败: ' + e.message);
      }
    };

    $('mapNameSelect').addEventListener('change', () => {
      if ($('mapNameSelect').value) {
        $('mapNameCustom').value = '';
      }
    });

    $('mapNameCustom').addEventListener('input', () => {
      if (($('mapNameCustom').value || '').trim()) {
        $('mapNameSelect').value = '';
      }
    });

    $('btnApplyDatum').onclick = async () => {
      const o = origin();
      if (!Number.isFinite(o.lat) || !Number.isFinite(o.lon)) {
        setStatus('原点无效：请填写有效的 Lat / Lon');
        return;
      }
      map.setView([o.lat, o.lon], map.getZoom());
      let msg = `原点已应用 ${o.lat.toFixed(6)}, ${o.lon.toFixed(6)} (yaw ${o.yaw_deg}°)`;
      try {
        const health = await KeepoutApi.health();
        if (health && health.navsat_ready) {
          const r = await KeepoutApi.setDatum(o.lat, o.lon, o.yaw_deg);
          msg += '；' + (r.message || 'SetDatum OK');
        }
      } catch (_) {
        /* ignore */
      }
      setStatus(msg);
    };

    $('btnGps').onclick = async () => {
      try {
        const g = await KeepoutApi.gps();
        if (g.lat == null) {
          setStatus('暂无 GPS (/gps/fix)');
          return;
        }
        if (gpsMarker) map.removeLayer(gpsMarker);
        gpsMarker = L.circleMarker([g.lat, g.lon], {
          radius: 7,
          color: '#3d9cf0',
          fillColor: '#3d9cf0',
          fillOpacity: 0.9,
        }).addTo(map);
        map.setView([g.lat, g.lon]);
        setStatus(`GPS ${g.lat.toFixed(6)}, ${g.lon.toFixed(6)}`);
        if (robotTracker) {
          robotTracker.updateDisplay({ available: true, gps_ok: true, lat: g.lat, lon: g.lon, mode: 'tile' });
        }
      } catch (e) {
        setStatus('GPS: ' + e.message);
      }
    };

    $('btnSave').onclick = async () => {
      try {
        const mapName = requireMapName();
        const notify = $('notifyOnSave').checked;
        if (getMapMode() === 'pgm') {
          const figures = drawer.toMapList(mapName);
          if (!figures.length) throw new Error('没有可保存的图形');
          const r = await KeepoutApi.saveMap(mapName, figures, notify);
          await refreshMapNameOptions(mapName);
          setStatus(r.message || `已保存 ${r.count} 个图形`);
          return;
        }
        const figures = drawer.toWgs84List();
        if (!figures.length) throw new Error('没有可保存的图形');
        const r = await KeepoutApi.saveWgs84(mapName, {
          origin: origin(),
          figures,
          use_ros: true,
          notify_nav2: notify,
        });
        await refreshMapNameOptions(mapName);
        setStatus(r.message || `已保存 ${r.count} 个图形`);
      } catch (e) {
        setStatus('保存失败: ' + e.message);
      }
    };

    $('btnLoad').onclick = async () => {
      try {
        const mapName = requireMapName();
        if (getMapMode() === 'pgm') {
          const raw = await KeepoutApi.loadMap(mapName);
          const list = (raw || []).map((f) => MapCoords.figureFromMapApi(f));
          drawer.loadFigures(list);
          setStatus(`已加载 ${list.length} 个图形 (${mapName})`);
          return;
        }
        const list = await KeepoutApi.loadWgs84(mapName, {
          origin: origin(),
          use_ros: true,
        });
        drawer.loadFigures(list);
        setStatus(`已加载 ${list.length} 个图形 (${mapName})`);
      } catch (e) {
        setStatus('加载失败: ' + e.message);
      }
    };
  }

  async function init() {
    let health = null;
    try {
      appCfg = await KeepoutApi.config();
      health = await KeepoutApi.health();
      setStatus('已连接 API' + (health.ros_enabled ? ' · ROS' : ''));
    } catch (e) {
      setStatus('API 未就绪: ' + e.message + '（可先启动 control_center_api）');
      appCfg = {};
    }

    const originCfg = appCfg.default_origin || {};
    $('originLat').value = originCfg.lat != null ? originCfg.lat : 38.161479;
    $('originLon').value = originCfg.lon != null ? originCfg.lon : -122.454630;
    $('originYaw').value = originCfg.yaw_deg != null ? originCfg.yaw_deg : 0;

    initTileMap(appCfg);
    wireUi();
    await refreshMapNameOptions();
    await refreshPgmMapOptions();
    await initRobotTracking(appCfg, health);
  }

  init();
})();
