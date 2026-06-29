"""
GPS map widget: Leaflet.js map in QWebEngineView.
Uses folium to generate the base HTML then injects JavaScript for live
marker updates without reloading the page.
"""

from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel
from PyQt6.QtCore    import Qt, QUrl
from PyQt6.QtGui     import QFont

_WEBENGINE_ERROR = ''
try:
    from PyQt6.QtWebEngineWidgets import QWebEngineView
    _HAS_WEBENGINE = True
except Exception as _e:
    # On Windows, a DLL load failure raises OSError, not ImportError.
    # Catch everything so the rest of the UI still launches.
    _HAS_WEBENGINE = False
    _WEBENGINE_ERROR = str(_e)

try:
    import folium
    _HAS_FOLIUM = True
except ImportError:
    _HAS_FOLIUM = False

from core.packet_decoder import TelemetryData

# Minimum update interval: don't spam the JS bridge more than 2 Hz
_UPDATE_INTERVAL_S = 0.5


def _build_map_html() -> tuple[str, str]:
    """
    Returns (html_string, map_var_name).
    Uses folium if available, otherwise a bare Leaflet.js template.
    """
    if _HAS_FOLIUM:
        m       = folium.Map(location=[0, 0], zoom_start=2, prefer_canvas=True)
        map_var = m.get_name()

        js = f"""
        <script>
        (function() {{
            var _rkt  = null;
            var _lnch = null;
            var _path = L.polyline([], {{color:'#00ff88', weight:2, opacity:0.85}}).addTo({map_var});
            var _pts  = [];
            var _set  = false;

            window.setLaunchSite = function(lat, lon) {{
                if (_lnch) {map_var}.removeLayer(_lnch);
                _lnch = L.circleMarker([lat, lon], {{
                    radius:9, color:'#ff4444', fillColor:'#ff4444', fillOpacity:0.9
                }}).bindTooltip('Launch site').addTo({map_var});
                {map_var}.setView([lat, lon], 16);
                _pts = []; _path.setLatLngs([]); _set = true;
            }};

            window.updateRocket = function(lat, lon) {{
                if (!_set) window.setLaunchSite(lat, lon);
                if (_rkt) {map_var}.removeLayer(_rkt);
                _rkt = L.circleMarker([lat, lon], {{
                    radius:6, color:'#00ff88', fillColor:'#00ff88', fillOpacity:1
                }}).addTo({map_var});
                _pts.push([lat, lon]);
                _path.setLatLngs(_pts);
            }};
        }})();
        </script>
        """
        m.get_root().html.add_child(folium.Element(js))
        return m.get_root().render(), map_var

    # Fallback: pure Leaflet template (no folium dependency)
    html = """<!DOCTYPE html>
<html><head>
<meta charset="utf-8"/>
<link rel="stylesheet"
      href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<style>body{margin:0}#map{width:100vw;height:100vh}</style>
</head><body>
<div id="map"></div>
<script>
var map = L.map('map').setView([0,0],2);
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',
    {attribution:'© OSM'}).addTo(map);
var _rkt=null,_lnch=null,_set=false;
var _path=L.polyline([],{color:'#00ff88',weight:2}).addTo(map);
var _pts=[];
window.setLaunchSite=function(lat,lon){
    if(_lnch)map.removeLayer(_lnch);
    _lnch=L.circleMarker([lat,lon],{radius:9,color:'#ff4444',
        fillColor:'#ff4444',fillOpacity:0.9}).addTo(map);
    map.setView([lat,lon],16);_pts=[];_path.setLatLngs([]);_set=true;};
window.updateRocket=function(lat,lon){
    if(!_set)window.setLaunchSite(lat,lon);
    if(_rkt)map.removeLayer(_rkt);
    _rkt=L.circleMarker([lat,lon],{radius:6,color:'#00ff88',
        fillColor:'#00ff88',fillOpacity:1}).addTo(map);
    _pts.push([lat,lon]);_path.setLatLngs(_pts);};
</script></body></html>"""
    return html, ''


class GPSMap(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._launch_set    = False
        self._last_update   = 0.0

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        if not _HAS_WEBENGINE:
            msg = (
                'GPS map unavailable — Qt WebEngine failed to load.\n\n'
                'Ensure PyQt6 and PyQt6-WebEngine are the same version:\n'
                '  pip install --upgrade PyQt6 PyQt6-WebEngine\n\n'
                + (_WEBENGINE_ERROR if _WEBENGINE_ERROR else '')
            )
            lbl = QLabel(msg)
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            lbl.setWordWrap(True)
            lbl.setStyleSheet('color:#8E8E93; font-size:11px; padding: 24px;')
            layout.addWidget(lbl)
            self._web = None
            return

        self._web = QWebEngineView()
        html, _   = _build_map_html()
        self._web.setHtml(html, QUrl('about:blank'))
        layout.addWidget(self._web)

    def update_data(self, data: TelemetryData) -> None:
        if self._web is None:
            return
        if not data.has_gps_fix:
            return

        import time
        now = time.time()
        if now - self._last_update < _UPDATE_INTERVAL_S:
            return
        self._last_update = now

        lat, lon = data.lat, data.lon

        if not self._launch_set:
            self._web.page().runJavaScript(f'setLaunchSite({lat}, {lon})')
            self._launch_set = True

        self._web.page().runJavaScript(f'updateRocket({lat}, {lon})')

    def reset(self) -> None:
        self._launch_set = False
        if self._web:
            html, _ = _build_map_html()
            self._web.setHtml(html, QUrl('about:blank'))
