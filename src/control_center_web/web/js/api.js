/**
 * HTTP client for control_center_api.
 */
const KeepoutApi = (() => {
  const base = () => (window.KEEPOUT_API_BASE || '');

  async function request(path, options = {}) {
    const res = await fetch(base() + path, {
      headers: { 'Content-Type': 'application/json', ...(options.headers || {}) },
      ...options,
    });
    let body = null;
    const text = await res.text();
    try {
      body = text ? JSON.parse(text) : null;
    } catch {
      body = text;
    }
    if (!res.ok) {
      const detail = (body && (body.detail || body.message)) || text || res.statusText;
      throw new Error(typeof detail === 'string' ? detail : JSON.stringify(detail));
    }
    return body;
  }

  return {
    health: () => request('/api/health'),
    config: () => request('/api/config'),
    listMaps: () => request('/api/maps'),
    deleteMap: (mapName, notifyNav2 = false) =>
      request(
        `/api/maps/${encodeURIComponent(mapName)}?notify_nav2=${notifyNav2 ? 'true' : 'false'}`,
        { method: 'DELETE' }
      ),
    loadMap: (mapName) => request(`/api/maps/${encodeURIComponent(mapName)}/keepouts`),
    saveWgs84: (mapName, payload) =>
      request(`/api/maps/${encodeURIComponent(mapName)}/keepouts/wgs84`, {
        method: 'POST',
        body: JSON.stringify(payload),
      }),
    loadWgs84: (mapName, payload) =>
      request(`/api/maps/${encodeURIComponent(mapName)}/keepouts/wgs84/load`, {
        method: 'POST',
        body: JSON.stringify(payload),
      }),
    notify: (mapName) =>
      request('/api/nav2/notify', {
        method: 'POST',
        body: JSON.stringify({ map_name: mapName }),
      }),
    setDatum: (lat, lon, yaw_deg) =>
      request('/api/ros/set-datum', {
        method: 'POST',
        body: JSON.stringify({ lat, lon, yaw_deg }),
      }),
    gps: () => request('/api/gps'),
    robotPose: (origin, useRos = true) => {
      const q = new URLSearchParams({
        lat: String(origin.lat),
        lon: String(origin.lon),
        yaw_deg: String(origin.yaw_deg || 0),
        use_ros: useRos ? 'true' : 'false',
      });
      return request(`/api/robot/pose?${q.toString()}`);
    },
  };
})();
