#!/usr/bin/env python3

#### jwc 26-0124-0000
#### * copy "C:\12i-Db\Dropbox\09k-E32-SM\25-0517-1900-E32--OPENED\13i-T-CameraPlus-S3-NOW-Ubuntu22_BmaxB1Pro--25-0505-0730-NOW\25-1123-1700-E32_TCameraPlusS3-AprilTag-SerialToMicrobit-HttpsCorsToGDevelop\examples\Camera_Screen_AprilTag__Serial_With_Microbit--Esp32_Client_Websocket-NOW\02-Ubuntu-Server_Hub\02-ubuntu_server_websocket-NOW.py" 
#### ** "c:\12i-Db\Dropbox\09k-E32-SM\25-0517-1900-E32--OPENED\26-0105-0500-Waveshare-E32S3_TouchDisplay2p0In_Cam-PioarduinoIde-NOW\src\09_ZA-Stage_02-ServerHub_WebSockets\02-ubuntu_server_websocket-NOW.py"

"""
Pure WebSocket Server for ESP32 AprilTag & GDevelop Communication
Uses flask-sock for reliable WebSocket support

Architecture:
    ESP32 <<-- WebSocket <<-- Python Server -->> WebSocket -->> GDevelop
    
Features:
    - Reliable WebSocket using flask-sock
    - Real-time AprilTag data streaming
    - GDevelop native WebSocketClient compatible
    - Lower memory usage on ESP32
    - Simple JSON message format
    - Backward compatible HTTP endpoints
    
MIGRATION NOTE (25-1202-0340):
    - Switched from simple-websocket to flask-sock
    - simple-websocket had compatibility issues (400 errors)
    - flask-sock is more reliable and works with Flask 3.x
"""

from flask import Flask, request, jsonify, make_response
from flask_cors import CORS
from flask_sock import Sock
import time
import json
from datetime import datetime
import threading
import logging
import re

# ============================================================================
# CONFIGURATION
# ============================================================================
SERVER_PORT = 5000
SERVER_HOST = '0.0.0.0'

# ============================================================================
# VIDEO PERFORMANCE MONITORING CONFIGURATION [jwc 25-1209-1600]
# ============================================================================
# These constants are OPTIONAL overrides for performance monitoring.
# If set to None, the system auto-detects target FPS from actual traffic.
#
# ⚙️ AUTO-DETECT MODE (Recommended):
#    Set ESP32_VIDEO_INTERVAL_MS_EXPECTED = None
#    Server learns target FPS from incoming video stream (median of last 20 frames)
#    Benefits: Zero configuration, adapts to ESP32 changes automatically
#
# 🔧 MANUAL OVERRIDE MODE:
#    Set ESP32_VIDEO_INTERVAL_MS_EXPECTED = 1000 (or your configured value)
#    Server compares actual FPS to this fixed target
#    Benefits: Useful for testing/debugging, explicit performance targets
#
# Current setting: AUTO-DETECT (None = learns from traffic)
#
MILLISECONDS_PER_SECOND = 1000.0  # Conversion constant (ms to seconds)
ESP32_VIDEO_INTERVAL_MS_EXPECTED = None  # None = auto-detect, 1000 = 1.0 FPS manual target

# ============================================================================
# SECURITY CONFIGURATION
# ============================================================================
# Authentication token - MUST MATCH ESP32's AUTH_TOKEN
# Generate new token: python3 -c "import secrets; print(secrets.token_urlsafe(32))"
#### jwc 25-1202-1120 o AUTH_TOKEN = "your_secret_token_change_this_12345"  # TODO: Change this!
AUTH_TOKEN = "Jesus333!!!"  # jwc 25-1202-1120 Matches ESP32

# WARNING: This token protects against unauthorized access, but data is still
# transmitted in plain text over ws://. For production, use VPN or wss://
# SECURITY NOTE: Change this to a unique random token for production!

# ============================================================================
# DATA STRUCTURES
# ============================================================================

class TagData:
    """AprilTag data structure"""
    def __init__(self):
        self.smartcam_ip = ""
        self.tag_id = 0
        self.camera_name = ""
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.x_cm = 0.0
        self.y_cm = 0.0
        self.z_cm = 0.0
        self.tag_size_percent = 0.0
        self.distance_cm = 0.0
        self.timestamp = 0

    def to_dict(self):
        return {
            'smartcam_ip': self.smartcam_ip,
            'tag_id': self.tag_id,
            'camera_name': self.camera_name,
            'yaw': self.yaw,
            'pitch': self.pitch,
            'roll': self.roll,
            'x_cm': self.x_cm,
            'y_cm': self.y_cm,
            'z_cm': self.z_cm,
            'tag_size_percent': self.tag_size_percent,
            'distance_cm': self.distance_cm,
            'timestamp': self.timestamp
        }

# ============================================================================
# GLOBAL STATE
# ============================================================================

# ============================================================================
# WEBSOCKET CLIENT TRACKING [jwc 25-1217-0550]
# ============================================================================
# websocket_clients: Dictionary storing all active WebSocket connections
# - Structure: {'esp32': [ws1, ws2, ...], 'gdevelop': [ws1, ws2, ...], 'viewers': [...]}
# - Each list contains WebSocket connection objects
# - Updated when clients connect/disconnect
#
# websocket_lock: Thread synchronization lock (CRITICAL FOR THREAD SAFETY)
# WHY NEEDED:
#   Flask runs each HTTP request and WebSocket connection in separate threads
#   Multiple threads can try to modify websocket_clients simultaneously:
#   - Thread A: ESP32 connecting, adding to list
#   - Thread B: GDevelop disconnecting, removing from list  
#   - Thread C: Broadcasting message, iterating through list
#   Without lock: Race conditions → corrupted lists, crashes
#   With lock: Only one thread accesses websocket_clients at a time
#
# USAGE PATTERN:
#   with websocket_lock:
#       # Safe to read/modify websocket_clients here
#       websocket_clients['esp32'].append(ws)
#
websocket_clients = {
    'esp32': [],       # ESP32 camera clients
    'gdevelop': [],    # GDevelop game clients
    'viewers': []      # Other viewer clients
}
websocket_lock = threading.Lock()  # MUST acquire before accessing websocket_clients

# Latest data
latest_apriltag_data = None
latest_video_frame = None  # Will store (jpeg_bytes, timestamp)
latest_video_frame_lock = threading.Lock()  # Thread-safe access to video frame

# Video FPS tracking [jwc 25-1206-1430]
video_frame_timestamps = []  # Last 10 frame timestamps for FPS calculation
MAX_FPS_SAMPLES = 10
calculated_fps = 0.0
video_fps_lock = threading.Lock()  # Thread-safe FPS calculation

# Video frame dimensions tracking [jwc 25-1207-0130]
actual_frame_width = 0
actual_frame_height = 0
frame_dimensions_lock = threading.Lock()

# Video performance metrics (jwc 25-1209-1310 - FPS optimization debugging)
class VideoPerformanceMetrics:
    """Track detailed video streaming performance for optimization"""
    def __init__(self):
        self.frame_intervals = []  # Time between consecutive frames
        self.frame_sizes = []      # Frame sizes in bytes
        self.processing_times = [] # Time to process each frame
        self.last_frame_time = 0
        self.total_frames = 0
        self.dropped_frames = 0
        self.lock = threading.Lock()
        self.MAX_SAMPLES = 20  # Keep last 20 measurements
    
    def record_frame(self, size_bytes, processing_time_ms):
        """Record frame reception with detailed metrics"""
        current_time = time.time()
        
        with self.lock:
            # Calculate interval since last frame
            if self.last_frame_time > 0:
                interval = current_time - self.last_frame_time
                self.frame_intervals.append(interval)
                if len(self.frame_intervals) > self.MAX_SAMPLES:
                    self.frame_intervals.pop(0)
            
            self.last_frame_time = current_time
            self.total_frames += 1
            
            # Track frame size
            self.frame_sizes.append(size_bytes)
            if len(self.frame_sizes) > self.MAX_SAMPLES:
                self.frame_sizes.pop(0)
            
            # Track processing time
            self.processing_times.append(processing_time_ms)
            if len(self.processing_times) > self.MAX_SAMPLES:
                self.processing_times.pop(0)
    
    def get_stats(self):
        """Get comprehensive performance statistics"""
        with self.lock:
            if not self.frame_intervals:
                return {
                    'avg_interval': 0, 'min_interval': 0, 'max_interval': 0,
                    'jitter': 0, 'avg_fps': 0, 'target_fps': 0,
                    'avg_size': 0, 'min_size': 0, 'max_size': 0,
                    'avg_processing': 0, 'max_processing': 0,
                    'total_frames': 0, 'dropped_frames': 0,
                    'efficiency': 0, 'recommendation': 'WAITING_FOR_DATA'
                }
            
            # Interval statistics (seconds)
            avg_interval = sum(self.frame_intervals) / len(self.frame_intervals)
            min_interval = min(self.frame_intervals)
            max_interval = max(self.frame_intervals)
            
            # Calculate jitter (standard deviation of intervals)
            mean_interval = avg_interval
            variance = sum((x - mean_interval) ** 2 for x in self.frame_intervals) / len(self.frame_intervals)
            jitter = variance ** 0.5
            
            # FPS calculations with dynamic target detection [jwc 25-1209-1600]
            avg_fps = 1.0 / avg_interval if avg_interval > 0 else 0
            
            # Determine target FPS: auto-detect (median) or manual override
            if ESP32_VIDEO_INTERVAL_MS_EXPECTED is None:
                # AUTO-DETECT MODE: Use median interval as target (robust to outliers)
                if len(self.frame_intervals) >= 10:
                    # Enough data - use median
                    sorted_intervals = sorted(self.frame_intervals)
                    median_interval = sorted_intervals[len(sorted_intervals) // 2]
                    target_interval = median_interval
                    target_source = "auto_detected"
                else:
                    # Not enough data yet - use current average
                    target_interval = avg_interval
                    target_source = "insufficient_data"
                
                target_fps = 1.0 / target_interval if target_interval > 0 else 0
                
                # Efficiency = consistency (how stable is delivery vs median?)
                # Compare average deviation from median
                if len(self.frame_intervals) >= 10:
                    deviation_from_median = abs(avg_interval - median_interval)
                    consistency_pct = (1.0 - (deviation_from_median / median_interval)) * 100
                    efficiency = max(0, min(100, consistency_pct))  # Clamp 0-100%
                else:
                    efficiency = 50.0  # Neutral until we have enough data
            else:
                # MANUAL OVERRIDE MODE: Use configured target
                target_interval = ESP32_VIDEO_INTERVAL_MS_EXPECTED / MILLISECONDS_PER_SECOND
                target_fps = MILLISECONDS_PER_SECOND / ESP32_VIDEO_INTERVAL_MS_EXPECTED
                target_source = "configured"
                
                # Efficiency = how close actual FPS is to configured target
                efficiency = (avg_fps / target_fps * 100) if target_fps > 0 else 0
            
            # Size statistics (bytes)
            avg_size = sum(self.frame_sizes) / len(self.frame_sizes)
            min_size = min(self.frame_sizes)
            max_size = max(self.frame_sizes)
            
            # Processing statistics (milliseconds)
            avg_processing = sum(self.processing_times) / len(self.processing_times)
            max_processing = max(self.processing_times)
            
            # Generate recommendation
            recommendation = self._generate_recommendation(
                avg_fps, target_fps, jitter, avg_processing, efficiency
            )
            
            return {
                'avg_interval': round(avg_interval, 3),
                'min_interval': round(min_interval, 3),
                'max_interval': round(max_interval, 3),
                'jitter': round(jitter, 3),
                'avg_fps': round(avg_fps, 3),
                'target_fps': round(target_fps, 1),
                'avg_size': int(avg_size),
                'min_size': int(min_size),
                'max_size': int(max_size),
                'avg_processing': round(avg_processing, 2),
                'max_processing': round(max_processing, 2),
                'total_frames': self.total_frames,
                'dropped_frames': self.dropped_frames,
                'efficiency': round(efficiency, 1),
                'recommendation': recommendation
            }
    
    def _generate_recommendation(self, avg_fps, target_fps, jitter, avg_processing, efficiency):
        """Generate tuning recommendation based on metrics"""
        if efficiency > 95:
            return "OPTIMAL - System performing excellently"
        elif efficiency > 85:
            return "GOOD - Minor tuning could improve performance"
        elif efficiency > 70:
            if jitter > 0.5:
                return "UNSTABLE - High jitter detected, consider increasing interval"
            else:
                return "FAIR - Consider adjusting ESP32 send interval"
        else:
            if avg_processing > 50:
                return "SLOW - Server processing bottleneck, optimize code"
            elif jitter > 1.0:
                return "UNSTABLE - Network issues, increase ESP32 interval significantly"
            else:
                return "POOR - Check ESP32 configuration and network quality"

# Global performance tracker
video_perf = VideoPerformanceMetrics()

# Legacy FIFO queue (HTTP compatibility)
tagData_List_Fifo = []
tagData_List_Fifo_Count_Base1 = 0
tagData_List_Fifo_Count_MAX_Base1 = 50
tagData_List_Fifo_Index_Head_Base0 = 0

# Statistics
stats = {
    'total_connections': 0,
    'active_connections': 0,
    'apriltag_events': 0,
    'video_frames': 0,
    'start_time': time.time(),
    'esp32_connected': False,
    'esp32_connect_time': 0,  # Track when ESP32 connected
    'gdevelop_clients': 0
}

# ESP32 connection history (jwc 25-1215-1700 - Track all connection sessions)
esp32_connection_history = []  # List of {connect_time, disconnect_time, duration_seconds} dicts

# Thread-safe lock for esp32_connection_history [jwc 25-1215-1700]
# WHY NEEDED: Flask runs multiple threads simultaneously:
#   - WebSocket threads write connection/disconnection events to history
#   - HTTP endpoint threads read history when users request /connection_history
#   - Multiple ESP32 devices could connect/disconnect concurrently
# WITHOUT LOCK: Race conditions cause data corruption (e.g., reading list while it's being modified)
# WITH LOCK: Only one thread can access esp32_connection_history at a time, ensuring data consistency
esp32_history_lock = threading.Lock()

MAX_CONNECTION_HISTORY = 20  # Keep last 20 connection sessions

# ============================================================================
# NETWORK LAG MONITORING [jwc 25-1207-2110]
# ============================================================================
# Tracks timing between AprilTag (WebSocket) and Video (HTTP) to ensure
# no interference/conflict. WebSocket needs 3s breathing room for stability.

class NetworkTimingMonitor:
    """Monitor timing between AprilTag WebSocket and Video HTTP uploads"""
    def __init__(self):
        self.last_apriltag_time = 0.0
        self.last_video_time = 0.0
        self.gaps = []  # Store recent gaps between video and apriltag
        self.MAX_GAPS = 20  # Keep last 20 measurements
        self.SAFETY_THRESHOLD_SEC = 3.0  # WebSocket needs 3s breathing room
        self.lock = threading.Lock()
    
    def record_apriltag(self):
        """Record AprilTag WebSocket message timestamp"""
        current_time = time.time()
        with self.lock:
            self.last_apriltag_time = current_time
    
    def record_video(self):
        """Record Video HTTP upload timestamp and calculate gap"""
        current_time = time.time()
        with self.lock:
            self.last_video_time = current_time
            
            # Calculate gap from last AprilTag message (if any)
            if self.last_apriltag_time > 0:
                gap = abs(current_time - self.last_apriltag_time)
                self.gaps.append(gap)
                if len(self.gaps) > self.MAX_GAPS:
                    self.gaps.pop(0)
                
                # Check if gap is below safety threshold
                if gap >= self.SAFETY_THRESHOLD_SEC:
                    status = "✅ SAFE"
                    comparison = f"gap >= {self.SAFETY_THRESHOLD_SEC}s"
                else:
                    status = "⚠️  TIGHT"
                    comparison = f"gap < {self.SAFETY_THRESHOLD_SEC}s"
                
                # Calculate statistics
                min_gap = min(self.gaps) if self.gaps else 0
                max_gap = max(self.gaps) if self.gaps else 0
                avg_gap = sum(self.gaps) / len(self.gaps) if self.gaps else 0
                
                # jwc 25-1209-1230 DISABLED: Timing monitor obsolete for WebSocket-only video
                # This was needed when HTTP video interfered with WebSocket AprilTag
                # Now both use WebSocket - no interference possible!
                # print(f"⏱️  TIMING: Video<<--AprilTag gap={gap:.2f}s | {status} ({comparison}) | Stats: min={min_gap:.2f}s avg={avg_gap:.2f}s max={max_gap:.2f}s")
    
    def get_stats(self):
        """Get timing statistics"""
        with self.lock:
            if not self.gaps:
                return {
                    'current_gap': 0.0,
                    'min_gap': 0.0,
                    'avg_gap': 0.0,
                    'max_gap': 0.0,
                    'safety_threshold': self.SAFETY_THRESHOLD_SEC,
                    'status': 'NO_DATA'
                }
            
            current_gap = self.gaps[-1] if self.gaps else 0
            return {
                'current_gap': round(current_gap, 2),
                'min_gap': round(min(self.gaps), 2),
                'avg_gap': round(sum(self.gaps) / len(self.gaps), 2),
                'max_gap': round(max(self.gaps), 2),
                'safety_threshold': self.SAFETY_THRESHOLD_SEC,
                'status': 'SAFE' if current_gap >= self.SAFETY_THRESHOLD_SEC else 'TIGHT'
            }

# Global timing monitor instance
timing_monitor = NetworkTimingMonitor()

# ============================================================================
# FLASK APP SETUP
# ============================================================================

app = Flask(__name__)
CORS(app, origins="*", supports_credentials=True)
sock = Sock(app)  # Initialize flask-sock

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================

def fix_json_quotes(message):
    """
    Fix JSON with single quotes to use double quotes.
    GDevelop may send JSON with single quotes, which is invalid JSON.
    This converts it to valid JSON format.
    """
    # Simply replace single quotes with double quotes
    # This handles the common case where GDevelop sends {'key':'value'}
    # and converts it to {"key":"value"}
    fixed = message.replace("'", '"')
    return fixed

# ============================================================================
# SECURITY FUNCTIONS
# ============================================================================

def validate_auth_token(data):
    """
    Validate authentication token from client
    
    Returns: (is_valid, error_message)
    """
    token = data.get('data', {}).get('auth_token', '')
    
    if not token:
        return (False, 'Missing auth_token')
    
    if token != AUTH_TOKEN:
        return (False, 'Invalid auth_token')
    
    return (True, '')

# ============================================================================
# WEBSOCKET HANDLERS
# ============================================================================

def broadcast_to_gdevelop(message_dict):
    """Broadcast message to all GDevelop clients"""
    message_json = json.dumps(message_dict)
    
    # Compact single-line debug print with ALL info preserved
    event_name = message_dict.get('event', 'unknown')
    data_info = ""
    if event_name == 'apriltag_data':
        data_info = f" tag_id={message_dict.get('tag_id', 0)}, x={message_dict.get('x_cm', 0):.1f}, y={message_dict.get('y_cm', 0):.1f}, z={message_dict.get('z_cm', 0):.1f}"
    print(f"*** SvHub -->> GDeve: ({len(websocket_clients['gdevelop'])} clients) Event={event_name},{data_info} JSON={message_json}")
    
    with websocket_lock:
        disconnected = []
        for ws in websocket_clients['gdevelop']:
            try:
                ws.send(message_json)
                print(f"      ✅ Delivered")
            except Exception as e:
                print(f"      ❌ Error: {e}")
                disconnected.append(ws)
        
        # Remove disconnected clients
        for ws in disconnected:
            websocket_clients['gdevelop'].remove(ws)
            stats['gdevelop_clients'] = len(websocket_clients['gdevelop'])

def handle_esp32_message(ws, message):
    """Handle message from ESP32"""
    global latest_apriltag_data
    
    try:
        data = json.loads(message)
        event = data.get('event', '')
        
        if event == 'identify':
            # ESP32 identifying itself - validate auth token
            is_valid, error_msg = validate_auth_token(data)
            
            if not is_valid:
                auth_fail_msg = {
                    'event': 'auth_failed',
                    'error': error_msg,
                    'message': 'Authentication required - check your auth_token'
                }
                print(f"*** SvHub <<-- Esp32: identify (AUTH FAILED: {error_msg})")
                print(f"*** SvHub -->> Esp32: {json.dumps(auth_fail_msg)}")
                ws.send(json.dumps(auth_fail_msg))
                return  # Don't add to clients list
            
            with websocket_lock:
                if ws not in websocket_clients['esp32']:
                    websocket_clients['esp32'].append(ws)
                    stats['esp32_connected'] = True
                    stats['esp32_connect_time'] = time.time()  # Track connection time
                    
                    # Record connection start in history [jwc 25-1215-1700]
                    with esp32_history_lock:
                        esp32_connection_history.append({
                            'connect_time': stats['esp32_connect_time'],
                            'disconnect_time': None,  # Still connected
                            'duration_seconds': None  # Will be calculated on disconnect
                        })
                        # Keep only last MAX_CONNECTION_HISTORY sessions
                        if len(esp32_connection_history) > MAX_CONNECTION_HISTORY:
                            esp32_connection_history.pop(0)
            
            esp32_name = data.get('data', {}).get('camera_name', 'ESP32-Unknown')
            
            identify_success_msg = {'event': 'identify_success', 'client_type': 'esp32'}
            print(f"*** SvHub <<-- Esp32: identify | 📹 ESP32 CLIENT IDENTIFIED ✅ Camera={esp32_name}, Auth=PASSED, ESP32_Clients={len(websocket_clients['esp32'])}")
            print(f"*** SvHub -->> Esp32: {json.dumps(identify_success_msg)}")
            ws.send(json.dumps(identify_success_msg))
            
        elif event == 'apriltag_data':
            # Received AprilTag data from ESP32
            # Support both nested (legacy) and flat (new) formats
            if 'data' in data and isinstance(data['data'], dict):
                # Legacy nested format: {"event": "apriltag_data", "data": {...}}
                payload = data['data']
            else:
                # New flat format: {"event": "apriltag_data", "tag_id": 5, ...}
                # Extract all fields except 'event'
                payload = {k: v for k, v in data.items() if k != 'event'}
            
            payload['server_timestamp'] = time.time()
            
            # Store as latest
            latest_apriltag_data = payload
            
            # Push to legacy FIFO
            tag_data = TagData()
            tag_data.smartcam_ip = payload.get('smartcam_ip', '')
            tag_data.tag_id = payload.get('tag_id', 0)
            tag_data.camera_name = payload.get('camera_name', '')
            tag_data.yaw = payload.get('yaw', 0.0)
            tag_data.pitch = payload.get('pitch', 0.0)
            tag_data.roll = payload.get('roll', 0.0)
            tag_data.x_cm = payload.get('x_cm', 0.0)
            tag_data.y_cm = payload.get('y_cm', 0.0)
            tag_data.z_cm = payload.get('z_cm', 0.0)
            tag_data.tag_size_percent = payload.get('tag_size_percent', 0.0)
            tag_data.distance_cm = payload.get('distance_cm', 0.0)
            tag_data.timestamp = payload['server_timestamp']
            AprilTag_List_Fifo_Push(tag_data)
            
            # Broadcast to GDevelop (keep flat format)
            broadcast_message = {
                'event': 'apriltag_data',
                **payload  # Spread operator - keeps flat structure
            }
            broadcast_to_gdevelop(broadcast_message)
            
            stats['apriltag_events'] += 1
            
            # Record AprilTag timing [jwc 25-1207-2110]
            timing_monitor.record_apriltag()
            
            print(f"*** SvHub <<-- Esp32: apriltag_data | 📡 ID={payload.get('tag_id')}, Pos=({payload.get('x_cm', 0):.1f},{payload.get('y_cm', 0):.1f},{payload.get('z_cm', 0):.1f})cm | GDevelop_clients={stats['gdevelop_clients']}")
            
            # Acknowledge to ESP32
            ack_msg = {'event': 'apriltag_ack', 'status': 'received'}
            print(f"*** SvHub -->> Esp32: (ACK) {json.dumps(ack_msg)}")
            ws.send(json.dumps(ack_msg))
            
        elif event == 'video_frame':
            # Received video frame
            stats['video_frames'] += 1
            print(f"*** SvHub <<-- Esp32: video_frame | 📹 Frame #{stats['video_frames']}")
            
        elif event == 'ping':
            print(f"*** SvHub <<-- Esp32: ping")
            pong_msg = {'event': 'pong', 'timestamp': time.time()}
            print(f"*** SvHub -->> Esp32: (PONG) {json.dumps(pong_msg)}")
            ws.send(json.dumps(pong_msg))
            
    except json.JSONDecodeError:
        print(f"❌ Invalid JSON from ESP32: {message[:100]}")
    except Exception as e:
        print(f"❌ Error handling ESP32 message: {e}")

def handle_gdevelop_message(ws, message):
    """Handle message from GDevelop"""
    try:
        data = json.loads(message)
        event = data.get('event', '')
        
        if event == 'identify':
            # GDevelop identifying itself
            with websocket_lock:
                if ws not in websocket_clients['gdevelop']:
                    websocket_clients['gdevelop'].append(ws)
                    stats['gdevelop_clients'] = len(websocket_clients['gdevelop'])
            
            gdevelop_name = data.get('data', {}).get('name', 'GDevelop-Client')
            
            identify_success_msg = {'event': 'identify_success', 'client_type': 'gdevelop'}
            print(f"*** SvHub <<-- GDeve: identify | 🎮 GDEVELOP CLIENT IDENTIFIED ✅ Name={gdevelop_name}, GDevelop_Clients={stats['gdevelop_clients']}")
            print(f"*** SvHub -->> GDeve: {json.dumps(identify_success_msg)}")
            ws.send(json.dumps(identify_success_msg))
            
            # Send latest data if available (FLAT format)
            if latest_apriltag_data:
                latest_data_msg = {'event': 'apriltag_data', **latest_apriltag_data}
                print(f"*** SvHub -->> GDeve: (Latest Data) {json.dumps(latest_data_msg)}")
                ws.send(json.dumps(latest_data_msg))
                
        elif event == 'request_latest_data':
            print(f"*** SvHub <<-- GDeve: request_latest_data")
            # GDevelop requesting latest data (FLAT format)
            if latest_apriltag_data:
                latest_data_msg = {'event': 'apriltag_data', **latest_apriltag_data}
                print(f"*** SvHub -->> GDeve: (Requested Data) {json.dumps(latest_data_msg)}")
                ws.send(json.dumps(latest_data_msg))
            else:
                no_data_msg = {'event': 'no_data_available'}
                print(f"*** SvHub -->> GDeve: {json.dumps(no_data_msg)}")
                ws.send(json.dumps(no_data_msg))
                
        elif event == 'ping':
            print(f"*** SvHub <<-- GDeve: ping")
            pong_msg = {'event': 'pong', 'timestamp': time.time()}
            print(f"*** SvHub -->> GDeve: (PONG) {json.dumps(pong_msg)}")
            ws.send(json.dumps(pong_msg))
            
    except json.JSONDecodeError:
        print(f"❌ Invalid JSON from GDevelop: {message[:100]}")
    except Exception as e:
        print(f"❌ Error handling GDevelop message: {e}")

@sock.route('/websocket')
def websocket(ws):
    """WebSocket endpoint for all clients using flask-sock"""
    stats['total_connections'] += 1
    stats['active_connections'] += 1
    
    # Get client connection info
    client_ip = request.remote_addr
    client_user_agent = request.headers.get('User-Agent', 'Unknown')
    client_origin = request.headers.get('Origin', 'Unknown')
    client_host = request.headers.get('Host', 'Unknown')
    
    print(f"\n{'='*70}")
    print(f"🔌 NEW WebSocket CONNECTION")
    print(f"{'='*70}")
    print(f"📍 Client IP: {client_ip}")
    print(f"🌐 Origin: {client_origin}")
    print(f"🖥️  Host: {client_host}")
    print(f"🔍 User-Agent: {client_user_agent[:80]}..." if len(client_user_agent) > 80 else f"🔍 User-Agent: {client_user_agent}")
    print(f"📊 Total Connections: {stats['total_connections']}")
    print(f"📊 Active Connections: {stats['active_connections']}")
    print(f"{'='*70}\n")
    
    # Send welcome message
    welcome_msg = {
        'event': 'connection_success',
        'message': 'WebSocket connected - send identify event'
    }
    print(f"*** SvHub -->> Esp32: {json.dumps(welcome_msg)}")
    ws.send(json.dumps(welcome_msg))
    
    client_type = 'unknown'
    client_identifier = f"{client_ip}:unknown"
    
    #### jwc 26-0125-0600 NEW: Message counter for debugging hang issue
    message_count = 0
    
    #### jwc 26-0125-0730 FIX: Add socket timeout to prevent freeze on incomplete/corrupted messages
    import socket
    try:
        ws._sock.settimeout(5.0)
        print(f"*** SvHub: Socket timeout set to 5.0s for {client_ip}")
    except Exception as e:
        print(f"*** SvHub: WARNING - Could not set socket timeout: {e}")
    
    try:
        while True:
            #### jwc 26-0125-0600 DEBUG: Print before receive (detect if blocking here)
            print(f"*** SvHub: [WAIT] Message #{message_count + 1} from {client_ip}...")
            
            #### jwc 26-0125-0730 FIX: Wrap ws.receive() in try/except to handle timeouts and errors
            try:
                message = ws.receive()
            except socket.timeout:
                # Timeout - no complete message received in 5 seconds
                print(f"*** SvHub: [TIMEOUT] No message from {client_ip} for 5s (last msg #{message_count})")
                # Send ping to check if connection alive and clear stuck state
                try:
                    ping_msg = {'event': 'ping', 'timestamp': time.time()}
                    ws.send(json.dumps(ping_msg))
                    print(f"*** SvHub -->> Esp32 {client_ip}: Sent ping (keepalive/recovery)")
                except Exception as e:
                    print(f"*** SvHub: Ping failed, connection dead: {e}")
                    break
                continue  # Go back to waiting for next message
            except Exception as e:
                # Other receive errors (connection closed, corrupted data, etc.)
                print(f"*** SvHub: [ERROR] ws.receive() failed for {client_ip}: {e}")
                break
            
            #### jwc 26-0125-0600 DEBUG: Print after receive
            message_count += 1
            if message is None:
                print(f"*** SvHub <<-- Esp32 {client_ip}: [CLOSE] Received None after {message_count} messages")
                break
            
            #### jwc 26-0125-0600 DEBUG: Print message details
            msg_type = "BINARY" if isinstance(message, bytes) else "TEXT"
            msg_len = len(message) if message else 0
            msg_preview = message[:100] if isinstance(message, str) else f"<{msg_len} bytes>"
            print(f"*** SvHub <<-- Esp32 {client_ip}: [RX #{message_count}] Type={msg_type}, Len={msg_len}, Preview={msg_preview}")
            
            #### jwc 26-0125-0730 NEW: Validate JSON messages to detect corruption
            if isinstance(message, str):
                try:
                    # Try to parse JSON to ensure it's valid
                    test_parse = json.loads(message)
                    print(f"*** SvHub: [VALID] JSON parsed OK")
                except json.JSONDecodeError as e:
                    print(f"*** SvHub: [CORRUPT] Invalid JSON at position {e.pos}: {e.msg}")
                    print(f"*** SvHub: [CORRUPT] Message preview: {message[:200]}")
                    # Skip this corrupted message and continue
                    continue
            
            # ============================================================================
            # BINARY MESSAGE HANDLING [jwc 25-1209-1040]
            # ============================================================================
            # Check if message is binary (video frame) vs text (JSON)
            if isinstance(message, bytes):
                # Binary message - this is a video frame from ESP32
                global latest_video_frame, calculated_fps, actual_frame_width, actual_frame_height
                
                try:
                    jpeg_data = message
                    
                    if jpeg_data and len(jpeg_data) > 0:
                        current_time = time.time()
                        
                        # Extract frame dimensions from JPEG header
                        try:
                            # Find SOF0 marker (0xFFC0)
                            for i in range(len(jpeg_data) - 10):
                                if jpeg_data[i] == 0xFF and jpeg_data[i+1] == 0xC0:
                                    # SOF0 found - extract dimensions (big-endian)
                                    height = (jpeg_data[i+5] << 8) | jpeg_data[i+6]
                                    width = (jpeg_data[i+7] << 8) | jpeg_data[i+8]
                                    
                                    with frame_dimensions_lock:
                                        actual_frame_width = width
                                        actual_frame_height = height
                                    break
                        except Exception as e:
                            print(f"⚠️  Could not extract JPEG dimensions: {e}")
                        
                        # Store frame with timestamp (thread-safe)
                        with latest_video_frame_lock:
                            latest_video_frame = (jpeg_data, current_time)
                        
                        # Calculate FPS from timestamps
                        with video_fps_lock:
                            video_frame_timestamps.append(current_time)
                            if len(video_frame_timestamps) > MAX_FPS_SAMPLES:
                                video_frame_timestamps.pop(0)
                            
                            if len(video_frame_timestamps) >= 2:
                                time_span = video_frame_timestamps[-1] - video_frame_timestamps[0]
                                if time_span > 0:
                                    calculated_fps = (len(video_frame_timestamps) - 1) / time_span
                        
                        stats['video_frames'] += 1
                        
                        # Record video timing
                        timing_monitor.record_video()
                        
                        # Record performance metrics [jwc 25-1209-1310]
                        processing_time_ms = (time.time() - current_time) * 1000
                        video_perf.record_frame(len(jpeg_data), processing_time_ms)
                        
                        # Get current dimensions for logging
                        with frame_dimensions_lock:
                            width_log = actual_frame_width
                            height_log = actual_frame_height
                        
                        # Get performance stats for enhanced debug output
                        perf_stats = video_perf.get_stats()
                        
                        print(f"SvHub <<-- Esp32: video_frame (WebSocket Binary) | 📹 {len(jpeg_data)} bytes {width_log}x{height_log} (#{stats['video_frames']}) FPS={calculated_fps:.2f}")
                        print(f"      📊 PERF: Interval={perf_stats['avg_interval']:.3f}s (min={perf_stats['min_interval']:.3f}s max={perf_stats['max_interval']:.3f}s) Jitter={perf_stats['jitter']:.3f}s")
                        print(f"      📊 SIZE: {perf_stats['avg_size']}B avg (min={perf_stats['min_size']}B max={perf_stats['max_size']}B)")
                        print(f"      📊 PROC: {perf_stats['avg_processing']:.2f}ms avg (max={perf_stats['max_processing']:.2f}ms)")
                        print(f"      🎯 TUNE: Efficiency={perf_stats['efficiency']:.1f}% | {perf_stats['recommendation']}")
                        
                        # Optional: Send acknowledgment to ESP32
                        # ack_msg = {'event': 'video_ack', 'status': 'received', 'size': len(jpeg_data)}
                        # ws.send(json.dumps(ack_msg))
                        
                except Exception as e:
                    print(f"❌ Error processing binary video frame: {e}")
                
                # Skip text processing for binary messages
                continue
            
            # ============================================================================
            # TEXT MESSAGE HANDLING (JSON)
            # ============================================================================
            # Determine client type from first message
            try:
                # Fix single quotes to double quotes if needed
                fixed_message = fix_json_quotes(message)
                data = json.loads(fixed_message)
                if data.get('event') == 'identify':
                    client_type = data.get('data', {}).get('type', 'unknown')
            except:
                pass
            
            # Fix single quotes to double quotes if needed (do this once for all paths)
            fixed_message = fix_json_quotes(message)
            
            # Route message based on client type
            if client_type == 'esp32' or ws in websocket_clients['esp32']:
                handle_esp32_message(ws, fixed_message)
            elif client_type == 'gdevelop' or ws in websocket_clients['gdevelop']:
                handle_gdevelop_message(ws, fixed_message)
            else:
                # Try to parse and route (fixed_message already created above)
                try:
                    data = json.loads(fixed_message)
                    event = data.get('event', '')
                    
                    # Auto-detect and register GDevelop clients based on event patterns
                    if event in ['request_latest_data', 'event_FromGDevelop'] or (event not in ['apriltag_data', 'video_frame', 'identify']):
                        # This looks like a GDevelop client - auto-register it
                        if ws not in websocket_clients['gdevelop']:
                            with websocket_lock:
                                websocket_clients['gdevelop'].append(ws)
                                stats['gdevelop_clients'] = len(websocket_clients['gdevelop'])
                            
                            print(f"🎮 AUTO-DETECTED GDevelop CLIENT | IP={client_ip}, FirstEvent={event}, GDevelop_Clients={stats['gdevelop_clients']}")
                        
                        client_type = 'gdevelop'
                        handle_gdevelop_message(ws, fixed_message)
                    elif event in ['apriltag_data', 'video_frame']:
                        handle_esp32_message(ws, fixed_message)
                    else:
                        handle_gdevelop_message(ws, fixed_message)
                except Exception as e:
                    print(f"⚠️  Unknown message (parse error): {message[:100]}")
                    print(f"    Error: {e}")
                    print(f"    Attempted fix: {fix_json_quotes(message)[:100]}")
                    
    except Exception as e:
        print(f"❌ WebSocket error: {e}")
    finally:
        # Cleanup
        stats['active_connections'] -= 1
        
        disconnected_type = 'Unknown'
        with websocket_lock:
            for client_list_type in websocket_clients:
                if ws in websocket_clients[client_list_type]:
                    websocket_clients[client_list_type].remove(ws)
                    disconnected_type = client_list_type
                    if client_list_type == 'esp32':
                        disconnect_time = time.time()
                        stats['esp32_connected'] = False
                        
                        # Record disconnect in history [jwc 25-1215-1700]
                        if stats['esp32_connect_time'] > 0:
                            with esp32_history_lock:
                                # Update the most recent connection session
                                if esp32_connection_history and esp32_connection_history[-1]['disconnect_time'] is None:
                                    esp32_connection_history[-1]['disconnect_time'] = disconnect_time
                                    esp32_connection_history[-1]['duration_seconds'] = int(disconnect_time - esp32_connection_history[-1]['connect_time'])
                        
                        # DON'T reset esp32_connect_time! [jwc 25-1216-1540]
                        # Keeping the last connect time allows uptime endpoint to show
                        # how long ago the ESP32 was connected (useful for debugging)
                    elif client_list_type == 'gdevelop':
                        stats['gdevelop_clients'] = len(websocket_clients['gdevelop'])
        
        print(f"❌ CLIENT DISCONNECTED | IP={client_ip}, Type={disconnected_type.upper()}, ID={client_identifier}, Active={stats['active_connections']}, ESP32={len(websocket_clients['esp32'])}, GDevelop={stats['gdevelop_clients']}")

# ============================================================================
# LEGACY HTTP ENDPOINTS
# ============================================================================

def AprilTag_List_Fifo_Push(tag_data):
    """Add tag data to FIFO queue"""
    global tagData_List_Fifo_Index_Head_Base0, tagData_List_Fifo_Count_Base1
    
    if tagData_List_Fifo_Count_Base1 < tagData_List_Fifo_Count_MAX_Base1:
        tagData_List_Fifo.append(tag_data)
        tagData_List_Fifo_Count_Base1 += 1
    else:
        tagData_List_Fifo[tagData_List_Fifo_Index_Head_Base0] = tag_data
        tagData_List_Fifo_Index_Head_Base0 = (tagData_List_Fifo_Index_Head_Base0 + 1) % tagData_List_Fifo_Count_MAX_Base1

def AprilTag_List_Fifo_Pop():
    """Remove and return oldest tag data"""
    global tagData_List_Fifo_Index_Head_Base0, tagData_List_Fifo_Count_Base1
    
    if tagData_List_Fifo_Count_Base1 == 0:
        return None
    
    if len(tagData_List_Fifo) < tagData_List_Fifo_Count_MAX_Base1:
        tag_data = tagData_List_Fifo.pop(0)
        tagData_List_Fifo_Count_Base1 -= 1
    else:
        tag_data = tagData_List_Fifo[tagData_List_Fifo_Index_Head_Base0]
        tagData_List_Fifo_Index_Head_Base0 = (tagData_List_Fifo_Index_Head_Base0 + 1) % tagData_List_Fifo_Count_MAX_Base1
        tagData_List_Fifo_Count_Base1 -= 1
    
    return tag_data

@app.route('/client_e32__http_post_to_serverhub__smartcam_video_stream', methods=['POST'])
def receive_video_frame():
    """Receive and cache video frames from ESP32"""
    global latest_video_frame, calculated_fps, actual_frame_width, actual_frame_height
    
    try:
        # Get JPEG data from request
        jpeg_data = request.get_data()
        
        if jpeg_data and len(jpeg_data) > 0:
            current_time = time.time()
            
            # Extract frame dimensions from JPEG header [jwc 25-1207-0130]
            # JPEG SOF0 marker format: FF C0 [length] [precision] [height:2bytes] [width:2bytes]
            try:
                # Find SOF0 marker (0xFFC0)
                for i in range(len(jpeg_data) - 10):
                    if jpeg_data[i] == 0xFF and jpeg_data[i+1] == 0xC0:
                        # SOF0 found - extract dimensions (big-endian)
                        height = (jpeg_data[i+5] << 8) | jpeg_data[i+6]
                        width = (jpeg_data[i+7] << 8) | jpeg_data[i+8]
                        
                        with frame_dimensions_lock:
                            actual_frame_width = width
                            actual_frame_height = height
                        break
            except Exception as e:
                print(f"⚠️  Could not extract JPEG dimensions: {e}")
            
            # Store frame with timestamp (thread-safe)
            with latest_video_frame_lock:
                latest_video_frame = (jpeg_data, current_time)
            
            # Calculate FPS from timestamps [jwc 25-1206-1430]
            with video_fps_lock:
                video_frame_timestamps.append(current_time)
                if len(video_frame_timestamps) > MAX_FPS_SAMPLES:
                    video_frame_timestamps.pop(0)
                
                if len(video_frame_timestamps) >= 2:
                    time_span = video_frame_timestamps[-1] - video_frame_timestamps[0]
                    if time_span > 0:
                        calculated_fps = (len(video_frame_timestamps) - 1) / time_span
            
            stats['video_frames'] += 1
            
            # jwc 25-1209-1230 DISABLED: Timing monitor obsolete for WebSocket-only video
            # timing_monitor.record_video()
            
            # Get current dimensions for logging
            with frame_dimensions_lock:
                width_log = actual_frame_width
                height_log = actual_frame_height
            
            print(f"📹 Video frame cached: {len(jpeg_data)} bytes {width_log}x{height_log} (#{stats['video_frames']}) FPS={calculated_fps:.2f}")
            
            return jsonify({'status': 'success', 'size': len(jpeg_data)}), 200
        else:
            return jsonify({'status': 'error', 'message': 'No data received'}), 400
            
    except Exception as e:
        print(f"❌ Error receiving video frame: {e}")
        return jsonify({'status': 'error', 'message': str(e)}), 500

@app.route('/photo')
def serve_latest_frame():
    """Serve the most recent video frame as JPEG"""
    global latest_video_frame
    
    with latest_video_frame_lock:
        if latest_video_frame is None:
            # No frame available yet - return placeholder
            return "No frame available yet. Waiting for ESP32...", 404
        
        jpeg_data, timestamp = latest_video_frame
        age_seconds = time.time() - timestamp
        
    # Create response with JPEG data
    response = make_response(jpeg_data)
    response.headers['Content-Type'] = 'image/jpeg'
    response.headers['Cache-Control'] = 'no-cache, no-store, must-revalidate'
    response.headers['Pragma'] = 'no-cache'
    response.headers['Expires'] = '0'
    response.headers['X-Frame-Age'] = f'{age_seconds:.1f}s'
    
    return response

@app.route('/video')
def video_viewer():
    """Live video viewer page with auto-refresh"""
    html = f"""
    <!DOCTYPE html>
    <html>
    <head>
        <title>ESP32 Camera Viewer</title>
        <meta charset="utf-8">
        <style>
            body {{
                font-family: Arial, sans-serif;
                margin: 0;
                padding: 20px;
                background: #1a1a1a;
                color: white;
                display: flex;
                flex-direction: column;
                align-items: center;
                min-height: 100vh;
            }}
            .container {{
                max-width: 400px;
                width: 100%;
            }}
            h1 {{
                text-align: center;
                color: #4CAF50;
                margin-bottom: 10px;
            }}
            .info {{
                text-align: center;
                color: #999;
                margin-bottom: 20px;
                font-size: 14px;
            }}
            .video-container {{
                background: #000;
                border: 3px solid #4CAF50;
                border-radius: 10px;
                padding: 10px;
                box-shadow: 0 0 20px rgba(76, 175, 80, 0.3);
            }}
            img {{
                width: 100%;
                height: auto;
                display: block;
                border-radius: 5px;
            }}
            .stats {{
                margin-top: 20px;
                padding: 15px;
                background: #2a2a2a;
                border-radius: 10px;
                font-size: 14px;
            }}
            .help-section {{
                margin-top: 20px;
                padding: 15px;
                background: #2a2a2a;
                border-radius: 10px;
                font-size: 13px;
                line-height: 1.6;
            }}
            .help-section h3 {{
                color: #4CAF50;
                margin-top: 0;
                margin-bottom: 10px;
                font-size: 16px;
            }}
            .help-section .metric {{
                margin-bottom: 12px;
                padding: 8px;
                background: #1a1a1a;
                border-radius: 5px;
            }}
            .help-section .metric-name {{
                color: #4CAF50;
                font-weight: bold;
            }}
            .help-section .metric-values {{
                margin-top: 5px;
                padding-left: 15px;
                color: #999;
                font-size: 12px;
            }}
            .stats-row {{
                display: flex;
                justify-content: space-between;
                margin: 5px 0;
            }}
            .status {{
                display: inline-block;
                width: 10px;
                height: 10px;
                border-radius: 50%;
                background: #4CAF50;
                margin-right: 8px;
                animation: pulse 2s infinite;
            }}
            @keyframes pulse {{
                0%, 100% {{ opacity: 1; }}
                50% {{ opacity: 0.5; }}
            }}
        </style>
    </head>
    <body>
        <div class="container">
            <h1>📹 ESP32 Camera Live View</h1>
            <div class="info">
                <span class="status"></span>
                <span id="refreshInfo">Web-updates: Loading... | Stats 1s</span>
            </div>
            
            <div class="video-container">
                <img src="/photo?t={{{{new Date().getTime()}}}}" 
                     alt="ESP32 Camera Feed" 
                     id="cameraFeed"
                     onerror="this.src='data:image/svg+xml,%3Csvg xmlns=%22http://www.w3.org/2000/svg%22 width=%22400%22 height=%22300%22%3E%3Crect fill=%22%23333%22 width=%22400%22 height=%22300%22/%3E%3Ctext x=%2250%25%22 y=%2250%25%22 text-anchor=%22middle%22 fill=%22%23999%22 font-family=%22Arial%22 font-size=%2220%22%3EWaiting for camera...%3C/text%3E%3C/svg%3E'">
            </div>
            
            <div class="stats">
                <div class="stats-row">
                    <strong>🎥 Resolution:</strong>
                    <span id="resolution">Detecting...</span>
                </div>
                <div class="stats-row">
                    <strong>🎯 Target FPS:</strong>
                    <span id="targetFps">Calculating...</span>
                </div>
                <div class="stats-row">
                    <strong>⚡ Actual FPS:</strong>
                    <span id="actualFps">Calculating...</span>
                </div>
                <div class="stats-row">
                    <strong>📊 Efficiency:</strong>
                    <span id="efficiency">Calculating...</span>
                    <span id="efficiencyHelp" style="font-size: 11px; color: #999; margin-left: 8px;"></span>
                </div>
                <div class="stats-row">
                    <strong>🎯 Jitter:</strong>
                    <span id="jitter">Calculating...</span>
                    <span id="jitterHelp" style="font-size: 11px; color: #999; margin-left: 8px;"></span>
                </div>
                <div class="stats-row">
                    <strong>📦 Frame Size:</strong>
                    <span id="frameSize">Calculating...</span>
                </div>
                <div class="stats-row">
                    <strong>⚙️  Processing:</strong>
                    <span id="processing">Calculating...</span>
                    <span id="processingHelp" style="font-size: 11px; color: #999; margin-left: 8px;"></span>
                </div>
                <div class="stats-row">
                    <strong>📡 Server:</strong>
                    <span>{SERVER_HOST}:{SERVER_PORT}</span>
                </div>
                <div class="stats-row">
                    <strong>⏱️  Frame Age:</strong>
                    <span id="latency">Calculating...</span>
                </div>
                <div class="stats-row">
                    <strong>🔌 Connection Uptime:</strong>
                    <span id="uptime">Not connected</span>
                </div>
                <div class="stats-row" style="margin-top: 10px; padding-top: 10px; border-top: 1px solid #444;">
                    <strong>💡 Recommendation:</strong>
                    <span id="recommendation" style="font-size: 12px;">Calculating...</span>
                </div>
            </div>
            
            <div class="help-section">
                <h3>📊 ESP32 Connection History</h3>
                <div id="connectionHistory" style="font-size: 13px;">
                    <p style="color: #999;">Loading connection history...</p>
                </div>
            </div>
            
            
            <div class="help-section" style="background: #2d5016;">
                <h3>🎮 Video Frame Rate Control</h3>
                
                <div style="margin-bottom: 15px; padding: 10px; background: #1a1a1a; border-radius: 5px;">
                    <strong>Current Interval:</strong> <span id="currentInterval" style="color: #4CAF50;">1000ms</span>
                    <span id="currentFpsControl" style="color: #999;">(1.0 FPS)</span>
                </div>
                
                <div style="margin-bottom: 10px;">
                    <button onclick="adjustInterval(-500)" style="margin: 5px; padding: 8px 15px; background: #27ae60; border: none; color: white; border-radius: 5px; cursor: pointer;">⬆️ Faster (-500ms)</button>
                    <button onclick="adjustInterval(-100)" style="margin: 5px; padding: 8px 15px; background: #27ae60; border: none; color: white; border-radius: 5px; cursor: pointer;">⬆️ Faster (-100ms)</button>
                    <button onclick="adjustInterval(100)" style="margin: 5px; padding: 8px 15px; background: #e67e22; border: none; color: white; border-radius: 5px; cursor: pointer;">⬇️ Slower (+100ms)</button>
                    <button onclick="adjustInterval(500)" style="margin: 5px; padding: 8px 15px; background: #e67e22; border: none; color: white; border-radius: 5px; cursor: pointer;">⬇️ Slower (+500ms)</button>
                </div>
                
                <div style="margin-bottom: 10px;">
                    <strong style="display: block; margin-bottom: 5px;">Quick Presets:</strong>
                    <button onclick="setIntervalValue(500)" style="margin: 5px; padding: 8px 15px; background: #3498db; border: none; color: white; border-radius: 5px; cursor: pointer;">0.5s (2.0 FPS)</button>
                    <button onclick="setIntervalValue(1000)" style="margin: 5px; padding: 8px 15px; background: #3498db; border: none; color: white; border-radius: 5px; cursor: pointer;">1.0s (1.0 FPS)</button>
                    <button onclick="setIntervalValue(2000)" style="margin: 5px; padding: 8px 15px; background: #3498db; border: none; color: white; border-radius: 5px; cursor: pointer;">2.0s (0.5 FPS)</button>
                    <button onclick="setIntervalValue(5000)" style="margin: 5px; padding: 8px 15px; background: #3498db; border: none; color: white; border-radius: 5px; cursor: pointer;">5.0s (0.2 FPS)</button>
                </div>
                
                <div style="margin-top: 15px;">
                    <strong style="display: block; margin-bottom: 5px;">Custom Interval (100-10000ms):</strong>
                    <input type="range" id="intervalSlider" min="100" max="10000" value="1000" step="100" style="width: 70%; vertical-align: middle;">
                    <button onclick="setIntervalFromSlider()" style="margin-left: 10px; padding: 8px 15px; background: #9b59b6; border: none; color: white; border-radius: 5px; cursor: pointer;">Apply</button>
                </div>
                
                <div style="margin-top: 10px; padding: 8px; background: #1a1a1a; border-radius: 5px; font-size: 12px; color: #bdc3c7;">
                    💡 <strong>Note:</strong> Changes are sent to ESP32 in real-time via WebSocket. Interval resets when ESP32 reboots.
                </div>
            </div>
            
            <div class="help-section">
                <h3>📖 Performance Metrics Explained</h3>
                
                <div class="metric">
                    <span class="metric-name">🎯 Jitter</span> - Timing consistency between frames
                    <div class="metric-values">
                        • <strong>Frame Interval:</strong> Time from start of one frame to the start of the next frame<br>
                        • <strong>What it is:</strong> Standard deviation of frame intervals (how much timing varies)<br>
                        • <strong>Why it matters:</strong> Low jitter = smooth, predictable stream; High jitter = choppy, unpredictable<br>
                        • <strong>Good values:</strong><br>
                        &nbsp;&nbsp;- <span style="color:#3498db">Excellent:</span> &lt;0.1s (100ms) - Very smooth, professional quality<br>
                        &nbsp;&nbsp;- <span style="color:#27ae60">Good:</span> 0.1-0.3s - Acceptable, minor variations<br>
                        &nbsp;&nbsp;- <span style="color:#f39c12">OK:</span> 0.3-0.5s - Noticeable but usable<br>
                        &nbsp;&nbsp;- <span style="color:#e74c3c">High:</span> &gt;0.5s - Choppy, needs optimization<br>
                        • <strong>How to improve:</strong> Increase ESP32 send interval, reduce resolution, lower JPEG quality
                    </div>
                </div>
                
                <div class="metric">
                    <span class="metric-name">📊 Efficiency</span> - How well actual performance matches target
                    <div class="metric-values">
                        • <strong>What it is:</strong> Percentage of how close actual FPS is to target FPS<br>
                        • <strong>Why it matters:</strong> Shows if ESP32 can maintain configured frame rate<br>
                        • <strong>Calculation:</strong> (Actual FPS / Target FPS) × 100%<br>
                        • <strong>Good values:</strong><br>
                        &nbsp;&nbsp;- <span style="color:#3498db">Optimal:</span> &gt;95% - System performing excellently<br>
                        &nbsp;&nbsp;- <span style="color:#27ae60">Good:</span> 85-95% - Minor tuning could help<br>
                        &nbsp;&nbsp;- <span style="color:#f39c12">Fair:</span> 70-85% - Consider adjusting send interval<br>
                        &nbsp;&nbsp;- <span style="color:#e74c3c">Poor:</span> &lt;70% - Network or configuration issues<br>
                        • <strong>Example:</strong> Target=1.0 FPS, Actual=0.85 FPS → Efficiency=85%<br>
                        • <strong>How to improve:</strong> Increase send interval for more realistic target
                    </div>
                </div>
                
                <div class="metric">
                    <span class="metric-name">⚙️  Processing</span> - Server-side frame processing time
                    <div class="metric-values">
                        • <strong>What it is:</strong> Time server takes to receive and store each frame (milliseconds)<br>
                        • <strong>Why it matters:</strong> Shows if server is a bottleneck (usually it's not!)<br>
                        • <strong>Good values:</strong><br>
                        &nbsp;&nbsp;- <span style="color:#3498db">Fast:</span> &lt;1ms - Server has plenty of headroom<br>
                        &nbsp;&nbsp;- <span style="color:#27ae60">Good:</span> 1-10ms - Normal, no issues<br>
                        &nbsp;&nbsp;- <span style="color:#f39c12">OK:</span> 10-50ms - Acceptable but watch it<br>
                        &nbsp;&nbsp;- <span style="color:#e74c3c">Slow:</strong> &gt;50ms - Server bottleneck, optimize code<br>
                        • <strong>Typical:</strong> 0.1-1ms on modern hardware<br>
                        • <strong>Note:</strong> Usually NOT the bottleneck - ESP32 or network usually is
                    </div>
                </div>
                
                <div class="metric">
                    <span class="metric-name">🔧 Quick Fixes</span> - Common optimizations
                    <div class="metric-values">
                        <strong>If Jitter is HIGH (>0.5s):</strong><br>
                        1. Increase ESP32 video interval (500ms → 1000ms)<br>
                        2. Reduce image resolution (240x240 → 176x144)<br>
                        3. Lower JPEG quality (10 → 6)<br><br>
                        
                        <strong>If Efficiency is LOW (&lt;70%):</strong><br>
                        1. ESP32 can't keep up - increase interval<br>
                        2. Check WiFi signal strength<br>
                        3. Reduce frame size/quality<br><br>
                        
                        <strong>If Processing is SLOW (>50ms):</strong><br>
                        1. Server overloaded - close other apps<br>
                        2. Upgrade server hardware<br>
                        3. Check network speed (rare issue)
                    </div>
                </div>
            </div>
        </div>
        
        <script>
            let currentIntervalMs = 0;  // Initialize to 0 (will be loaded from ESP32)
            
            // Fetch actual interval from ESP32 on page load
            fetch('/video_fps')
                .then(response => response.json())
                .then(data => {{
                    if (data.interval_ms > 0) {{
                        currentIntervalMs = data.interval_ms;
                        updateIntervalDisplay();
                        
                        // Also update the refresh info display immediately [jwc 25-1217-0615]
                        const refreshRate = (currentIntervalMs * 0.9 / 1000).toFixed(1);
                        document.getElementById('refreshInfo').textContent = 
                            'Web-updates: Video ' + refreshRate + 's (auto) | Stats 1s';
                        
                        console.log('✅ Loaded actual ESP32 interval:', currentIntervalMs, 'ms');
                    }}
                }})
                .catch(err => console.log('Initial interval fetch error:', err));
            
            function adjustInterval(delta) {{
                let newInterval = currentIntervalMs + delta;
                setIntervalValue(newInterval);
            }}
            
            function setIntervalValue(intervalMs) {{
                // Clamp to valid range
                intervalMs = Math.max(100, Math.min(10000, intervalMs));
                
                fetch('/set_video_interval', {{
                    method: 'POST',
                    headers: {{'Content-Type': 'application/json'}},
                    body: JSON.stringify({{interval_ms: intervalMs}})
                }})
                .then(response => response.json())
                .then(data => {{
                    if (data.status === 'success') {{
                        currentIntervalMs = intervalMs;
                        updateIntervalDisplay();
                        console.log('✅ Video interval updated:', intervalMs, 'ms');
                    }} else {{
                        console.error('❌ Failed to update interval:', data.message);
                        alert('Failed: ' + data.message);
                    }}
                }})
                .catch(err => {{
                    console.error('❌ Network error:', err);
                    alert('Network error - ESP32 may not be connected');
                }});
            }}
            
            function setIntervalFromSlider() {{
                let slider = document.getElementById('intervalSlider');
                setIntervalValue(parseInt(slider.value));
            }}
            
            function updateIntervalDisplay() {{
                let fps = (1000.0 / currentIntervalMs).toFixed(2);
                document.getElementById('currentInterval').textContent = currentIntervalMs + 'ms';
                document.getElementById('currentFpsControl').textContent = '(' + fps + ' FPS)';
                document.getElementById('intervalSlider').value = currentIntervalMs;
            }}
            
            // Update display on page load
            updateIntervalDisplay();
            
            // Dynamic video refresh based on ESP32 frame rate [jwc 25-1215-1900]
            let videoRefreshInterval = 2000;  // Default fallback (2s)
            let videoRefreshTimeout = null;
            
            function updateVideoRefreshRate() {{
                fetch('/video_fps')
                    .then(response => response.json())
                    .then(data => {{
                        if (data.interval_ms > 0) {{
                            // Refresh at 90% of ESP32 interval (slightly faster to catch all frames)
                            // Min 500ms to avoid hammering server, max 5000ms to stay responsive
                            const newInterval = Math.max(500, Math.min(5000, data.interval_ms * 0.9));
                            
                            if (Math.abs(newInterval - videoRefreshInterval) > 50) {{
                                // Only update if change is significant (>50ms)
                                videoRefreshInterval = newInterval;
                                const refreshRate = (videoRefreshInterval / 1000).toFixed(1);
                                document.getElementById('refreshInfo').textContent = 
                                    'Web-updates: Video ' + refreshRate + 's (auto) | Stats 1s';
                                console.log('📹 Video refresh adjusted to ' + videoRefreshInterval + 'ms (ESP32 sends at ' + data.interval_ms + 'ms)');
                            }}
                        }}
                    }})
                    .catch(err => console.log('Video refresh rate update error:', err));
            }}
            
            function refreshVideo() {{
                var img = document.getElementById('cameraFeed');
                img.src = '/photo?t=' + new Date().getTime();
                
                // Schedule next refresh based on current dynamic interval
                videoRefreshTimeout = setTimeout(refreshVideo, videoRefreshInterval);
            }}
            
            // Initialize dynamic video refresh
            updateVideoRefreshRate();  // Get initial ESP32 rate
            refreshVideo();  // Start refresh loop
            
            // Re-check ESP32 interval every 10 seconds (adapts to user changes)
            setInterval(updateVideoRefreshRate, 10000);
            
            // Update all stats every second (jwc 25-1206-1430, enhanced jwc 25-1207-0130, jwc 25-1209-1430)
            function updateStats() {{
                // Fetch frame age from headers
                fetch('/photo')
                    .then(response => {{
                        const age = response.headers.get('X-Frame-Age');
                        if (age) {{
                            document.getElementById('latency').textContent = age;
                        }}
                    }})
                    .catch(err => console.log('Frame age fetch error:', err));
                
                // Fetch basic FPS data
                fetch('/video_fps')
                    .then(response => response.json())
                    .then(data => {{
                        // Update resolution
                        if (data.resolution && data.resolution !== 'Unknown') {{
                            document.getElementById('resolution').textContent = 
                                data.resolution + ' pixels';
                        }} else {{
                            document.getElementById('resolution').textContent = 'Waiting for frame...';
                        }}
                        
                        // Update target FPS
                        if (data.target_fps > 0) {{
                            document.getElementById('targetFps').textContent = 
                                data.target_fps.toFixed(1) + ' FPS (' + data.interval_ms + 'ms interval)';
                        }} else {{
                            document.getElementById('targetFps').textContent = 'Calculating...';
                        }}
                        
                        // Update actual FPS
                        document.getElementById('actualFps').textContent = 
                            data.fps.toFixed(2) + ' FPS';
                    }})
                    .catch(err => console.log('Stats fetch error:', err));
                
                // Fetch comprehensive performance stats from /video_stats
                fetch('/video_stats')
                    .then(response => response.json())
                    .then(data => {{
                        // Update efficiency with color coding and help text
                        const efficiency = data.efficiency || 0;
                        let effColor = '#e74c3c';  // Red for poor
                        let effHelp = 'POOR';
                        
                        if (efficiency > 95) {{
                            effColor = '#3498db';  // Blue for optimal
                            effHelp = 'OPTIMAL';
                        }} else if (efficiency > 85) {{
                            effColor = '#27ae60';  // Green for good
                            effHelp = 'GOOD';
                        }} else if (efficiency > 70) {{
                            effColor = '#f39c12';  // Orange for fair
                            effHelp = 'FAIR';
                        }}
                        
                        document.getElementById('efficiency').innerHTML = 
                            '<span style="color:' + effColor + '">' + efficiency.toFixed(1) + '%</span>';
                        document.getElementById('efficiencyHelp').innerHTML = 
                            '<span style="color:' + effColor + '">(' + effHelp + ')</span>';
                        
                        // Update jitter with help text
                        const jitter = data.jitter || 0;
                        let jitterHelp = '';
                        let jitterColor = '#27ae60';  // Green by default
                        
                        if (jitter < 0.1) {{
                            jitterHelp = '(EXCELLENT)';
                            jitterColor = '#3498db';
                        }} else if (jitter < 0.3) {{
                            jitterHelp = '(GOOD)';
                            jitterColor = '#27ae60';
                        }} else if (jitter < 0.5) {{
                            jitterHelp = '(OK)';
                            jitterColor = '#f39c12';
                        }} else {{
                            jitterHelp = '(HIGH)';
                            jitterColor = '#e74c3c';
                        }}
                        
                        document.getElementById('jitter').textContent = 
                            data.jitter ? data.jitter.toFixed(3) + 's' : 'N/A';
                        document.getElementById('jitterHelp').innerHTML = 
                            '<span style="color:' + jitterColor + '">' + jitterHelp + '</span>';
                        
                        // Update frame size
                        const avgSize = data.avg_size || 0;
                        const sizeKB = (avgSize / 1024).toFixed(1);
                        document.getElementById('frameSize').textContent = 
                            avgSize + 'B (' + sizeKB + 'KB) avg';
                        
                        // Update processing time with help text
                        const avgProc = data.avg_processing || 0;
                        let procHelp = '';
                        let procColor = '#27ae60';
                        
                        if (avgProc < 1) {{
                            procHelp = '(FAST)';
                            procColor = '#3498db';
                        }} else if (avgProc < 10) {{
                            procHelp = '(GOOD)';
                            procColor = '#27ae60';
                        }} else if (avgProc < 50) {{
                            procHelp = '(OK)';
                            procColor = '#f39c12';
                        }} else {{
                            procHelp = '(SLOW)';
                            procColor = '#e74c3c';
                        }}
                        
                        document.getElementById('processing').textContent = 
                            (data.avg_processing || 0).toFixed(2) + 'ms avg (max: ' + 
                            (data.max_processing || 0).toFixed(2) + 'ms)';
                        document.getElementById('processingHelp').innerHTML = 
                            '<span style="color:' + procColor + '">' + procHelp + '</span>';
                        
                        // Update recommendation with color coding
                        const rec = data.recommendation || 'Waiting for data...';
                        let recColor = '#bdc3c7';  // Gray default
                        
                        if (rec.includes('OPTIMAL')) {{
                            recColor = '#3498db';  // Blue
                        }} else if (rec.includes('GOOD')) {{
                            recColor = '#27ae60';  // Green
                        }} else if (rec.includes('FAIR')) {{
                            recColor = '#f39c12';  // Orange
                        }} else if (rec.includes('UNSTABLE') || rec.includes('SLOW') || rec.includes('POOR')) {{
                            recColor = '#e74c3c';  // Red
                        }}
                        
                        document.getElementById('recommendation').innerHTML = 
                            '<span style="color:' + recColor + '">' + rec + '</span>';
                    }})
                    .catch(err => console.log('Performance stats fetch error:', err));
                
                // Fetch connection uptime [jwc 25-1215-1100]
                fetch('/connection_uptime')
                    .then(response => response.json())
                    .then(data => {{
                        if (data.connected) {{
                            document.getElementById('uptime').textContent = data.uptime_formatted;
                            document.getElementById('uptime').style.color = '#27ae60';  // Green for connected
                        }} else {{
                            document.getElementById('uptime').textContent = 'Not connected';
                            document.getElementById('uptime').style.color = '#e74c3c';  // Red for disconnected
                        }}
                    }})
                    .catch(err => console.log('Uptime fetch error:', err));
                
                // Fetch connection history [jwc 25-1215-1700]
                fetch('/connection_history')
                    .then(response => response.json())
                    .then(data => {{
                        let historyHTML = '';
                        
                        if (data.sessions && data.sessions.length > 0) {{
                            historyHTML += '<div style="margin-bottom: 10px; color: #bdc3c7;">';
                            historyHTML += '<strong>Total Sessions:</strong> ' + data.total_sessions;
                            historyHTML += ' | <strong>Server Started:</strong> ' + data.server_start_time;
                            historyHTML += '</div>';
                            
                            historyHTML += '<table style="width: 100%; border-collapse: collapse; font-size: 12px;">';
                            historyHTML += '<tr style="background: #1a1a1a; border-bottom: 2px solid #4CAF50;">';
                            historyHTML += '<th style="padding: 8px; text-align: left; color: #4CAF50;">Status</th>';
                            historyHTML += '<th style="padding: 8px; text-align: left; color: #4CAF50;">Connected</th>';
                            historyHTML += '<th style="padding: 8px; text-align: left; color: #4CAF50;">Disconnected</th>';
                            historyHTML += '<th style="padding: 8px; text-align: left; color: #4CAF50;">Duration</th>';
                            historyHTML += '</tr>';
                            
                            // Reverse to show newest first
                            for (let i = data.sessions.length - 1; i >= 0; i--) {{
                                const session = data.sessions[i];
                                const isConnected = session.status === 'connected';
                                const statusColor = isConnected ? '#27ae60' : '#95a5a6';
                                const statusIcon = isConnected ? '🟢' : '⚪';
                                
                                historyHTML += '<tr style="border-bottom: 1px solid #444;">';
                                historyHTML += '<td style="padding: 8px; color: ' + statusColor + ';">' + statusIcon + ' ' + session.status + '</td>';
                                historyHTML += '<td style="padding: 8px; color: #bdc3c7;">' + session.connect_time + '</td>';
                                historyHTML += '<td style="padding: 8px; color: #bdc3c7;">' + session.disconnect_time + '</td>';
                                historyHTML += '<td style="padding: 8px; color: #bdc3c7;">' + session.duration_formatted + '</td>';
                                historyHTML += '</tr>';
                            }}
                            
                            historyHTML += '</table>';
                        }} else {{
                            historyHTML = '<p style="color: #999;">No connection history available</p>';
                        }}
                        
                        document.getElementById('connectionHistory').innerHTML = historyHTML;
                    }})
                    .catch(err => {{
                        console.log('Connection history fetch error:', err);
                        document.getElementById('connectionHistory').innerHTML = 
                            '<p style="color: #e74c3c;">Failed to load connection history</p>';
                    }});
            }}
            
            // Update stats immediately and then every second
            updateStats();
            setInterval(updateStats, 1000);
        </script>
    </body>
    </html>
    """
    return html

@app.route('/client_gdevelop_to_server__smartcam_data_get', methods=['GET', 'OPTIONS'])
def serve_gdevelop_data_http():
    """Legacy HTTP GET endpoint"""
    if request.method == 'OPTIONS':
        response = make_response('', 200)
        response.headers['Access-Control-Allow-Origin'] = '*'
        response.headers['Access-Control-Allow-Methods'] = 'GET, OPTIONS'
        response.headers['Access-Control-Allow-Headers'] = 'Content-Type'
        return response
    
    tag_data = AprilTag_List_Fifo_Pop()
    
    if tag_data is None:
        return jsonify({
            'smartcam_ip': '', 'tag_id': -1, 'camera_name': '',
            'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0,
            'x_cm': 0.0, 'y_cm': 0.0, 'z_cm': 0.0,
            'tag_size_percent': 0.0, 'distance_cm': 0.0,
            'timestamp': 0, 'list_remaining': 0
        })
    
    response = make_response(jsonify(tag_data.to_dict()))
    response.headers['Access-Control-Allow-Origin'] = '*'
    return response

@app.route('/')
def home():
    """Status web page"""
    uptime_min = int((time.time() - stats['start_time']) / 60)
    
    html = f"""
    <!DOCTYPE html>
    <html>
    <head>
        <title>flask-sock WebSocket Server</title>
        <meta http-equiv="refresh" content="5">
        <style>
            body {{ font-family: Arial; margin: 20px; background: #2c3e50; color: white; }}
            .container {{ max-width: 1200px; margin: 0 auto; }}
            .header {{ background: #34495e; padding: 20px; border-radius: 10px; margin-bottom: 20px; }}
            .status-grid {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 15px; }}
            .status-card {{ background: #34495e; padding: 20px; border-radius: 10px; text-align: center; }}
            .status-value {{ font-size: 2.5em; font-weight: bold; color: #3498db; }}
            .status-label {{ color: #bdc3c7; margin-top: 10px; }}
            .connected {{ color: #27ae60; }}
            .disconnected {{ color: #e74c3c; }}
            code {{ background: #2c3e50; padding: 5px; border-radius: 3px; }}
        </style>
    </head>
    <body>
        <div class="container">
            <div class="header">
                <h1>🚀 flask-sock WebSocket Server</h1>
                <p>Reliable WebSocket with Flask 3.x</p>
            </div>
            
            <div class="status-grid">
                <div class="status-card">
                    <div class="status-value {'connected' if stats['esp32_connected'] else 'disconnected'}">
                        {'✅' if stats['esp32_connected'] else '❌'}
                    </div>
                    <div class="status-label">ESP32</div>
                </div>
                
                <div class="status-card">
                    <div class="status-value">{stats['gdevelop_clients']}</div>
                    <div class="status-label">GDevelop</div>
                </div>
                
                <div class="status-card">
                    <div class="status-value">{stats['active_connections']}</div>
                    <div class="status-label">Connections</div>
                </div>
                
                <div class="status-card">
                    <div class="status-value">{stats['apriltag_events']}</div>
                    <div class="status-label">AprilTag Events</div>
                </div>
                
                <div class="status-card">
                    <div class="status-value">{uptime_min}</div>
                    <div class="status-label">Uptime (min)</div>
                </div>
            </div>
            
            <div style="background: #34495e; padding: 20px; border-radius: 10px; margin-top: 20px;">
                <h3>📡 WebSocket Endpoint</h3>
                <p><code>ws://localhost:{SERVER_PORT}/websocket</code></p>
                <p><code>wss://your-ngrok.ngrok-free.app/websocket</code></p>
                
                <h3>📨 Message Format (JSON)</h3>
                <pre style="background: #2c3e50; padding: 10px; border-radius: 5px;">
// Identify as ESP32
{{"event": "identify", "data": {{"type": "esp32"}}}}

// Send AprilTag data
{{"event": "apriltag_data", "data": {{"tag_id": 5, "x_cm": 10.5, ...}}}}

// Identify as GDevelop
{{"event": "identify", "data": {{"type": "gdevelop"}}}}
                </pre>
            </div>
        </div>
    </body>
    </html>
    """
    return html

@app.route('/status')
def status():
    """JSON status"""
    return jsonify({
        'server': 'online',
        'protocol': 'flask_sock_websocket',
        'uptime_seconds': int(time.time() - stats['start_time']),
        'websocket': {
            'active_connections': stats['active_connections'],
            'esp32_connected': stats['esp32_connected'],
            'gdevelop_clients': stats['gdevelop_clients']
        }
    })

@app.route('/video_fps')
def video_fps():
    """JSON endpoint for video FPS data [jwc 25-1206-1430] and dimensions [jwc 25-1207-0130]
    Updated [jwc 25-1209-1700]: Now uses dynamic target FPS from performance metrics (auto-detected)"""
    with video_fps_lock:
        current_fps = calculated_fps
    
    with frame_dimensions_lock:
        width = actual_frame_width
        height = actual_frame_height
    
    # Get dynamic target FPS from performance metrics (auto-detected from actual traffic)
    # This replaces the old hardcoded TARGET_INTERVAL_MS approach
    perf_stats = video_perf.get_stats()
    target_fps = perf_stats.get('target_fps', 0)
    
    # Calculate interval from target FPS for display purposes
    # 1000.0 = milliseconds per second (conversion factor: FPS → ms interval)
    # Example: 2.0 FPS → 1000.0 / 2.0 = 500ms interval
    interval_ms = int(1000.0 / target_fps) if target_fps > 0 else 0
    
    return jsonify({
        'fps': round(current_fps, 2),
        'target_fps': round(target_fps, 1),
        'interval_ms': interval_ms,
        'width': width,
        'height': height,
        'resolution': f'{width}x{height}' if width > 0 and height > 0 else 'Unknown'
    })

@app.route('/video_stats')
def video_stats():
    """JSON endpoint for comprehensive video performance statistics [jwc 25-1209-1310]"""
    perf_stats = video_perf.get_stats()
    
    # Add basic FPS and dimensions data
    with video_fps_lock:
        perf_stats['current_fps'] = round(calculated_fps, 2)
    
    with frame_dimensions_lock:
        perf_stats['width'] = actual_frame_width
        perf_stats['height'] = actual_frame_height
        perf_stats['resolution'] = f'{actual_frame_width}x{actual_frame_height}' if actual_frame_width > 0 else 'Unknown'
    
    return jsonify(perf_stats)

@app.route('/connection_uptime')
def connection_uptime():
    # JSON endpoint for ESP32 connection uptime [jwc 25-1215-1100]
    # Updated [jwc 25-1217-0610]: Check actual connection list instead of flag
    #
    # WHY: The stats['esp32_connected'] flag can get out of sync with reality
    # - If ESP32 briefly disconnects and reconnects, flag may be stale
    # - Checking the actual websocket_clients list is more reliable
    # - This fixes issue where uptime shows "Not connected" when video is still working
    #
    # THREAD SAFETY: We acquire websocket_lock to safely read the connection list
    
    # Check if there are any active ESP32 connections (more reliable than flag)
    with websocket_lock:  # LOCK: Safely read websocket_clients['esp32']
        connection_with_eps32_bool = len(websocket_clients['esp32']) > 0
    
    if connection_with_eps32_bool and stats['esp32_connect_time'] > 0:
        uptime_seconds = int(time.time() - stats['esp32_connect_time'])
        
        # Convert to days, hours, minutes
        days = uptime_seconds // 86400
        hours = (uptime_seconds % 86400) // 3600
        minutes = (uptime_seconds % 3600) // 60
        seconds = uptime_seconds % 60
        
        # Format string [jwc 25-1217-0750: Added seconds display]
        uptime_str = ""
        if days > 0:
            uptime_str += f"{days}d "
        if hours > 0 or days > 0:
            uptime_str += f"{hours}h "
        if minutes > 0 or hours > 0 or days > 0:
            uptime_str += f"{minutes}m "
        uptime_str += f"{seconds}s"
        
        return jsonify({
            'connected': True,
            'uptime_seconds': uptime_seconds,
            'uptime_formatted': uptime_str,
            'days': days,
            'hours': hours,
            'minutes': minutes,
            'seconds': seconds
        })
    else:
        return jsonify({
            'connected': False,
            'uptime_seconds': 0,
            'uptime_formatted': 'Not connected',
            'days': 0,
            'hours': 0,
            'minutes': 0,
            'seconds': 0
        })

@app.route('/connection_history')
def connection_history():
    """JSON endpoint for ESP32 connection history [jwc 25-1215-1700]
    IMPORTANT: Only returns PAST sessions (disconnected). Current connection shown in /connection_uptime"""
    with esp32_history_lock:
        history = []
        for session in esp32_connection_history:
            # ONLY include disconnected sessions in history [jwc 25-1216-1530]
            if session['disconnect_time'] is not None:
                connect_dt = datetime.fromtimestamp(session['connect_time'])
                disconnect_dt = datetime.fromtimestamp(session['disconnect_time'])
                
                # Calculate duration from stored timestamps (frozen at disconnect time)
                duration_sec = int(session['disconnect_time'] - session['connect_time'])
                
                entry = {
                    'connect_time': connect_dt.strftime('%Y-%m-%d %H:%M:%S'),
                    'connect_timestamp': session['connect_time'],
                    'disconnect_time': disconnect_dt.strftime('%Y-%m-%d %H:%M:%S'),
                    'disconnect_timestamp': session['disconnect_time'],
                    'duration_seconds': duration_sec,
                    'duration_formatted': f"{duration_sec // 60}m {duration_sec % 60}s",
                    'status': 'disconnected'
                }
                history.append(entry)
    
    return jsonify({
        'sessions': history,
        'total_sessions': len(history),
        'server_start_time': datetime.fromtimestamp(stats['start_time']).strftime('%Y-%m-%d %H:%M:%S')
    })

@app.route('/set_video_interval', methods=['POST'])
def set_video_interval():
    """Set ESP32 video frame send interval in real-time [jwc 25-1210-0700]"""
    data = request.get_json()
    interval_ms = data.get('interval_ms', 1000)
    
    # Validate range (100ms to 10s)
    if interval_ms < 100 or interval_ms > 10000:
        return jsonify({'status': 'error', 'message': 'Interval must be 100-10000ms'}), 400
    
    # Send command to ESP32 via WebSocket
    command = {
        'event': 'set_video_interval',
        'interval_ms': interval_ms
    }
    
    with websocket_lock:
        if len(websocket_clients['esp32']) > 0:
            for ws in websocket_clients['esp32']:
                try:
                    ws.send(json.dumps(command))
                    print(f"📤 SEND to ESP32: set_video_interval={interval_ms}ms ({1000.0/interval_ms:.2f} FPS)")
                except Exception as e:
                    print(f"❌ Failed to send interval command: {e}")
                    pass
            return jsonify({'status': 'success', 'interval_ms': interval_ms, 'fps': round(1000.0/interval_ms, 2)})
        else:
            print("⚠️  ESP32 not connected - cannot set video interval")
            return jsonify({'status': 'error', 'message': 'ESP32 not connected'}), 503

# ============================================================================
# MAIN
# ============================================================================

def print_startup_info():
    """Print startup information"""
    import socket
    
    hostname = socket.gethostname()
    local_ip = socket.gethostbyname(hostname)
    
    print("\n" + "="*70)
    print("🚀 flask-sock WebSocket Server Starting...")
    print("="*70)
    print(f"🖥️  Hostname: {hostname}")
    print(f"🌐 Local IP: {local_ip}")
    print(f"🔌 Port: {SERVER_PORT}")
    print("="*70)
    print("📡 WebSocket Endpoint:")
    print(f"   ws://{local_ip}:{SERVER_PORT}/websocket")
    print(f"   wss://your-ngrok.ngrok-free.app/websocket")
    print("="*70)
    print("🌐 HTTP Endpoints:")
    print(f"   http://{local_ip}:{SERVER_PORT}/")
    print(f"      └─ Server status dashboard (auto-refresh)")
    print(f"   http://{local_ip}:{SERVER_PORT}/video")
    print(f"      └─ 📹 Live camera video viewer (auto-refresh)")
    print(f"   http://{local_ip}:{SERVER_PORT}/photo")
    print(f"      └─ 📷 Latest camera photo (JPEG)")
    print(f"   http://{local_ip}:{SERVER_PORT}/status")
    print(f"      └─ JSON status API")
    print(f"   http://{local_ip}:{SERVER_PORT}/client_gdevelop_to_server__smartcam_data_get")
    print(f"      └─ Legacy HTTP polling endpoint for GDevelop")
    print("="*70)
    print("📨 WebSocket Message Format (JSON):")
    print('   ESP32 Identify:')
    print('      {{"event": "identify", "data": {{"type": "esp32", "auth_token": "..."}}}}')
    print('   GDevelop Identify:')
    print('      {{"event": "identify", "data": {{"type": "gdevelop"}}}}')
    print('   AprilTag Data:')
    print('      {{"event": "apriltag_data", "data": {{"tag_id": 5, ...}}}}')
    print('   Request Latest:')
    print('      {{"event": "request_latest_data"}}')
    print("="*70)
    print("💡 Quick Start:")
    print("   1. pip install -r 0a-requirements.txt")
    print("   2. (Optional) Start ngrok: ngrok http 5000")
    print("   3. Connect ESP32 to ws://[IP]:5000/websocket")
    print("   4. Connect GDevelop to ws://[IP]:5000/websocket")
    print(f"   5. View dashboard: http://{local_ip}:{SERVER_PORT}/")
    print("="*70)
    print("✅ Server ready and listening!")
    print("="*70 + "\n")

if __name__ == '__main__':
    log = logging.getLogger('werkzeug')
    log.setLevel(logging.ERROR)
    
    print_startup_info()
    
    try:
        app.run(host=SERVER_HOST, port=SERVER_PORT, debug=False)
    except KeyboardInterrupt:
        print("\n🛑 Server stopped")
    except Exception as e:
        print(f"❌ Error: {e}")
