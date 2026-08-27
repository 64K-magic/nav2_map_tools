/**
 * Keepout Edit frontend — tile map + draw tools + API save/load.
 */
(function () {
  const $ = (id) => document.getElementById(id);
  const statusEl = $('status');
  const figureList = $('figureList');

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

  function requireMapName() {
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

  function tileUrlTemplate(url) {
    // Leaflet uses {z}/{x}/{y}; GUI used {level}
    return url.replace(/\{level\}/g, '{z}');
  }

  let map;
  let tileLayer;
  let drawer;
  let gpsMarker;

  async function init() {
    let cfg = {};
    try {
      cfg = await KeepoutApi.config();
      setStatus('已连接 API');
    } catch (e) {
      setStatus('API 未就绪: ' + e.message + '（可先启动 keepout_edit_api）');
    }

    const originCfg = cfg.default_origin || {};
    const tileCfg = cfg.tile || {};
    $('originLat').value = originCfg.lat != null ? originCfg.lat : 38.161479;
    $('originLon').value = originCfg.lon != null ? originCfg.lon : -122.454630;
    $('originYaw').value = originCfg.yaw_deg != null ? originCfg.yaw_deg : 0;

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
      doubleClickZoom: true,
    });

    setTileLayer(defaultTileUrl);

    drawer = new DrawController(map, refreshFigureList);
    drawer.bind();

    wireUi();
    refreshFigureList();
    applyInflationFromUi();
    await refreshMapNameOptions();
  }

  function applyInflationFromUi() {
    if (!drawer) return;
    drawer.setInflationPreview(
      $('showInflationChk').checked,
      parseFloat($('inscribedRadius').value),
      parseFloat($('inflationRadius').value)
    );
  }

  function setTileLayer(url) {
    if (tileLayer) map.removeLayer(tileLayer);
    tileLayer = L.tileLayer(tileUrlTemplate(url), {
      maxZoom: 22,
      attribution: 'Tiles',
      crossOrigin: true,
    }).addTo(map);
  }

  function refreshFigureList(selectedIdx) {
    figureList.innerHTML = '';
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

    $('btnUndo').onclick = () => drawer.undo();
    $('btnClear').onclick = () => {
      if (confirm('清空当前所有图形？')) drawer.clear();
    };

    $('showInflationChk').addEventListener('change', applyInflationFromUi);
    $('inscribedRadius').addEventListener('change', applyInflationFromUi);
    $('inscribedRadius').addEventListener('input', applyInflationFromUi);
    $('inflationRadius').addEventListener('change', applyInflationFromUi);
    $('inflationRadius').addEventListener('input', applyInflationFromUi);

    $('btnGoCenter').onclick = () => {
      const o = origin();
      map.setView([o.lat, o.lon], map.getZoom());
    };

    $('btnRefreshMaps').onclick = () => refreshMapNameOptions();

    $('btnDeleteMap').onclick = async () => {
      const name = ($('mapNameSelect').value || '').trim();
      if (!name) {
        setStatus('请先在下拉框中选择要删除的地图');
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
      // Save/Load always use these fields for local ENU — no ROS required.
      map.setView([o.lat, o.lon], map.getZoom());
      let msg = `原点已应用 ${o.lat.toFixed(6)}, ${o.lon.toFixed(6)} (yaw ${o.yaw_deg}°)`;
      // Optional: if API started with --ros, also push SetDatum
      try {
        const health = await KeepoutApi.health();
        if (health && health.navsat_ready) {
          const r = await KeepoutApi.setDatum(o.lat, o.lon, o.yaw_deg);
          msg += '；' + (r.message || 'SetDatum OK');
        }
      } catch (_) {
        /* ignore — pure Python mode */
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
      } catch (e) {
        setStatus('GPS: ' + e.message);
      }
    };

    $('btnSave').onclick = async () => {
      try {
        const mapName = requireMapName();
        const figures = drawer.toWgs84List();
        if (!figures.length) throw new Error('没有可保存的图形');
        const r = await KeepoutApi.saveWgs84(mapName, {
          origin: origin(),
          figures,
          use_ros: true,
          notify_nav2: $('notifyOnSave').checked,
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

  init();
})();
