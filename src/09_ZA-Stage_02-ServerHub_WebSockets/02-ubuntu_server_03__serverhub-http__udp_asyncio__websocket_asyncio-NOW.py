#!/usr/bin/env python3

#### jwc 26-0127-0630
#### * Migrated from flask-sock to websockets async library
#### * Fixes freeze issue: flask-sock ws.receive() blocks forever on incomplete messages
#### * websockets library has proper timeout handling (no freeze!)
#### * NO ESP32 code changes needed (standard WebSocket protocol)

"""
Async WebSocket Server for ESP32 AprilTag & GDevelop Communication
Uses websockets library for reliable async WebSocket support with timeout handling

Architecture:
    ESP32 <<-- WebSocket <<-- Python Server -->> WebSocket -->> GDevelop
    
Features:
    - Async WebSocket using websockets library (fixes freeze issue!)
    - Proper timeout handling (no blocking forever!)
    - Real-time AprilTag data streaming
    - GDevelop native WebSocketClient compatible
    - Lower memory usage on ESP32
    - Simple JSON message format
    - HTTP endpoints for status/video
    
MIGRATION NOTE (26-0127-0630):
    - Switched from flask-sock to websockets (async-based)
    - flask-sock had critical freeze issue (ws.receive() blocks forever)
    - websockets has built-in timeout handling and proper error recovery
    - NO ESP32 code changes needed!
"""

import asyncio
import websockets
from flask import Flask, jsonify, request
from flask_cors import CORS
import json
import time
from datetime import datetime
import threading
import logging

# ============================================================================
# CONFIGURATION
# ============================================================================
SERVER_PORT = 5000       # HTTP endpoints (Flask) - Status dashboard, video viewer, etc.
WEBSOCKET_PORT = 5100    # WebSocket endpoint (websockets library) - ESP32 and GDevelop connect here
UDP_PORT = 5200          # UDP endpoint (asyncio) - ESP32 sends AprilTag data here (fire-and-forget!)
SERVER_HOST = '0.0.0.0'

#### jwc 26-0130-1130 NOTE: Using separate ports for HTTP, WebSocket, and UDP
# - HTTP (Flask): http://localhost:5000/ - Status dashboard, video viewer, metrics
# - WebSocket: ws://localhost:5100/ - Real-time AprilTag data streaming (bidirectional)
# - UDP: udp://localhost:5200/ - Fire-and-forget AprilTag data (ZERO memory leaks!)
# - ESP32 can use ANY protocol (WebSocket, HTTP POST, or UDP)

# ============================================================================
# SECURITY CONFIGURATION
# ============================================================================
# Authentication token - MUST MATCH ESP32's AUTH_TOKEN
AUTH_TOKEN = "Jesus333!!!"  # jwc 25-1202-1120 Matches ESP32

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

# WebSocket client tracking (thread-safe with asyncio)
websocket_clients = {
    'esp32': set(),      # ESP32 camera clients
    'gdevelop': set(),   # GDevelop game clients
    'viewers': set()     # Other viewer clients
}

# Latest data
latest_apriltag_data = None

# Statistics
stats = {
    'total_connections': 0,
    'active_connections': 0,
    'apriltag_events': 0,
    'apriltag_udp_events': 0,  # jwc 26-0130-1130 NEW: Track UDP packets
    'video_frames': 0,
    'start_time': time.time(),
    'esp32_connected': False,
    'esp32_udp_active': False,  # jwc 26-0130-1130 NEW: Track UDP activity
    'esp32_connect_time': 0,
    'gdevelop_clients': 0
}

#### jwc 26-0128-1440 NEW: Store event loop for async broadcast from Flask HTTP endpoint
websocket_loop = None

# ============================================================================
# UTILITY FUNCTIONS
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
# WEBSOCKET HANDLERS (ASYNC)
# ============================================================================

async def broadcast_to_gdevelop(message_dict):
    """Broadcast message to all GDevelop clients (async)"""
    message_json = json.dumps(message_dict)
    
    # Compact single-line debug print
    event_name = message_dict.get('event', 'unknown')
    data_info = ""
    if event_name == 'apriltag_data':
        data_info = f" tag_id={message_dict.get('tag_id', 0)}, x={message_dict.get('x_cm', 0):.1f}, y={message_dict.get('y_cm', 0):.1f}, z={message_dict.get('z_cm', 0):.1f}"
    print(f"*** ..... .... SvHub -->> GDeve: ({len(websocket_clients['gdevelop'])} clients) Event={event_name},{data_info}")
    
    # Broadcast to all GDevelop clients
    disconnected = set()
    for ws in websocket_clients['gdevelop'].copy():
        try:
            await ws.send(message_json)
            print(f"      ✅ Delivered")
        except Exception as e:
            print(f"      ❌ Error: {e}")
            disconnected.add(ws)
    
    # Remove disconnected clients
    websocket_clients['gdevelop'] -= disconnected
    stats['gdevelop_clients'] = len(websocket_clients['gdevelop'])

async def handle_esp32_message(ws, message):
    """Handle message from ESP32 (async)"""
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
                print(f"*** Esp32 -->> SvHub .... .....: identify (AUTH FAILED: {error_msg})")
                await ws.send(json.dumps(auth_fail_msg))
                return  # Don't add to clients list
            
            websocket_clients['esp32'].add(ws)
            stats['esp32_connected'] = True
            stats['esp32_connect_time'] = time.time()
            
            esp32_name = data.get('data', {}).get('camera_name', 'ESP32-Unknown')
            
            identify_success_msg = {'event': 'identify_success', 'client_type': 'esp32'}
            print(f"*** Esp32 -->> SvHub .... .....: identify | 📹 ESP32 CLIENT IDENTIFIED ✅ Camera={esp32_name}, Auth=PASSED, ESP32_Clients={len(websocket_clients['esp32'])}")
            await ws.send(json.dumps(identify_success_msg))
            
        elif event == 'apriltag_data':
            # Received AprilTag data from ESP32
            # Support both nested (legacy) and flat (new) formats
            if 'data' in data and isinstance(data['data'], dict):
                # Legacy nested format
                payload = data['data']
            else:
                # New flat format
                payload = {k: v for k, v in data.items() if k != 'event'}
            
            payload['server_timestamp'] = time.time()
            
            # Store as latest
            latest_apriltag_data = payload
            
            # Broadcast to GDevelop (keep flat format)
            broadcast_message = {
                'event': 'apriltag_data',
                **payload
            }
            await broadcast_to_gdevelop(broadcast_message)
            
            stats['apriltag_events'] += 1
            
            print(f"*** SvHub <<-- Esp32: apriltag_data | 📡 ID={payload.get('tag_id')}, Pos=({payload.get('x_cm', 0):.1f},{payload.get('y_cm', 0):.1f},{payload.get('z_cm', 0):.1f})cm | GDevelop_clients={stats['gdevelop_clients']}")
            
            # Acknowledge to ESP32
            ack_msg = {'event': 'apriltag_ack', 'status': 'received'}
            await ws.send(json.dumps(ack_msg))
            
        elif event == 'ping':
            print(f"*** SvHub <<-- Esp32: ping")
            pong_msg = {'event': 'pong', 'timestamp': time.time()}
            await ws.send(json.dumps(pong_msg))
            
    except json.JSONDecodeError:
        print(f"❌ Invalid JSON from ESP32: {message[:100]}")
    except Exception as e:
        print(f"❌ Error handling ESP32 message: {e}")

async def handle_gdevelop_message(ws, message):
    """Handle message from GDevelop (async)"""
    try:
        data = json.loads(message)
        event = data.get('event', '')
        
        if event == 'identify':
            # GDevelop identifying itself
            websocket_clients['gdevelop'].add(ws)
            stats['gdevelop_clients'] = len(websocket_clients['gdevelop'])
            
            gdevelop_name = data.get('data', {}).get('name', 'GDevelop-Client')
            
            identify_success_msg = {'event': 'identify_success', 'client_type': 'gdevelop'}
            print(f"*** SvHub <<-- GDeve: identify | 🎮 GDEVELOP CLIENT IDENTIFIED ✅ Name={gdevelop_name}, GDevelop_Clients={stats['gdevelop_clients']}")
            await ws.send(json.dumps(identify_success_msg))
            
            # Send latest data if available (FLAT format)
            if latest_apriltag_data:
                latest_data_msg = {'event': 'apriltag_data', **latest_apriltag_data}
                await ws.send(json.dumps(latest_data_msg))
                
        elif event == 'request_latest_data':
            print(f"*** SvHub <<-- GDeve: request_latest_data")
            # GDevelop requesting latest data (FLAT format)
            if latest_apriltag_data:
                latest_data_msg = {'event': 'apriltag_data', **latest_apriltag_data}
                await ws.send(json.dumps(latest_data_msg))
            else:
                no_data_msg = {'event': 'no_data_available'}
                await ws.send(json.dumps(no_data_msg))
                
        elif event == 'ping':
            print(f"*** SvHub <<-- GDeve: ping")
            pong_msg = {'event': 'pong', 'timestamp': time.time()}
            await ws.send(json.dumps(pong_msg))
            
    except json.JSONDecodeError:
        print(f"❌ Invalid JSON from GDevelop: {message[:100]}")
    except Exception as e:
        print(f"❌ Error handling GDevelop message: {e}")

#### jwc 26-0127-0630 NEW: Async WebSocket handler with proper timeout!
async def websocket_handler(websocket, path):
    """
    Main WebSocket connection handler (async)
    
    KEY IMPROVEMENT: Uses async for loop with built-in timeout handling
    - No more blocking forever on incomplete messages!
    - Proper error recovery
    - Clean connection cleanup
    """
    stats['total_connections'] += 1
    stats['active_connections'] += 1
    
    # Get client connection info
    client_ip = websocket.remote_address[0] if websocket.remote_address else 'Unknown'
    
    print(f"\n{'='*70}")
    print(f"🔌 NEW WebSocket CONNECTION")
    print(f"{'='*70}")
    print(f"📍 Client IP: {client_ip}")
    print(f"📊 Total Connections: {stats['total_connections']}")
    print(f"📊 Active Connections: {stats['active_connections']}")
    print(f"{'='*70}\n")
    
    # Send welcome message
    welcome_msg = {
        'event': 'connection_success',
        'message': 'WebSocket connected - send identify event'
    }
    await websocket.send(json.dumps(welcome_msg))
    
    client_type = 'unknown'
    
    try:
        #### jwc 26-0127-0630 KEY FIX: async for loop with built-in timeout!
        # This replaces flask-sock's ws.receive() which blocked forever
        # websockets library handles timeouts automatically via ping/pong
        async for message in websocket:
            # Determine client type from first message
            if client_type == 'unknown':
                try:
                    data = json.loads(message)
                    if data.get('event') == 'identify':
                        client_type = data.get('data', {}).get('type', 'unknown')
                except:
                    pass
            
            # Route message based on client type
            if client_type == 'esp32' or websocket in websocket_clients['esp32']:
                await handle_esp32_message(websocket, message)
            elif client_type == 'gdevelop' or websocket in websocket_clients['gdevelop']:
                await handle_gdevelop_message(websocket, message)
            else:
                # Try to auto-detect and route
                try:
                    data = json.loads(message)
                    event = data.get('event', '')
                    
                    # Auto-detect GDevelop clients
                    if event in ['request_latest_data', 'event_FromGDevelop']:
                        if websocket not in websocket_clients['gdevelop']:
                            websocket_clients['gdevelop'].add(websocket)
                            stats['gdevelop_clients'] = len(websocket_clients['gdevelop'])
                            print(f"🎮 AUTO-DETECTED GDevelop CLIENT | IP={client_ip}, FirstEvent={event}")
                        
                        client_type = 'gdevelop'
                        await handle_gdevelop_message(websocket, message)
                    elif event in ['apriltag_data', 'video_frame']:
                        await handle_esp32_message(websocket, message)
                    else:
                        await handle_gdevelop_message(websocket, message)
                except Exception as e:
                    print(f"⚠️  Unknown message (parse error): {message[:100]}")
                    print(f"    Error: {e}")
                    
    except websockets.exceptions.ConnectionClosed:
        print(f"🔌 Connection closed normally: {client_ip}")
    except Exception as e:
        print(f"❌ WebSocket error: {e}")
    finally:
        # Cleanup
        stats['active_connections'] -= 1
        
        disconnected_type = 'Unknown'
        for client_list_type in websocket_clients:
            if websocket in websocket_clients[client_list_type]:
                websocket_clients[client_list_type].remove(websocket)
                disconnected_type = client_list_type
                if client_list_type == 'esp32':
                    stats['esp32_connected'] = False
                elif client_list_type == 'gdevelop':
                    stats['gdevelop_clients'] = len(websocket_clients['gdevelop'])
        
        print(f"❌ CLIENT DISCONNECTED | IP={client_ip}, Type={disconnected_type.upper()}, Active={stats['active_connections']}, ESP32={len(websocket_clients['esp32'])}, GDevelop={stats['gdevelop_clients']}")

# ============================================================================
# UDP HANDLERS (ASYNC)
# ============================================================================

#### jwc 26-0130-1130 NEW: UDP Protocol Handler for ESP32 AprilTag Data
class UDPProtocol(asyncio.DatagramProtocol):
    """
    UDP Protocol Handler for ESP32 AprilTag Data
    
    KEY BENEFITS:
    - 🔥 Fire-and-forget (no connection overhead)
    - 🔥 Stateless (ZERO memory leaks!)
    - 🔥 Simple (no handshake, no TIME_WAIT)
    - 🔥 Fast (minimal latency)
    
    ESP32 sends JSON packets like:
    {
        "tag_id": 5,
        "x_cm": 10.5,
        "y_cm": 20.3,
        "z_cm": 30.1,
        ...
    }
    """
    
    def connection_made(self, transport):
        """Called when UDP socket is ready"""
        self.transport = transport
        print(f"✅ UDP server ready on port {UDP_PORT}")
    
    def datagram_received(self, data, addr):
        """
        Called when UDP packet is received from ESP32
        
        Args:
            data: Raw bytes from UDP packet
            addr: (ip, port) tuple of sender
        """
        global latest_apriltag_data, stats
        
        try:
            # Decode JSON from UDP packet
            message = data.decode('utf-8')
            apriltag_data = json.loads(message)
            
            # Add server timestamp and source IP
            apriltag_data['server_timestamp'] = time.time()
            apriltag_data['smartcam_ip'] = addr[0]
            
            # Store as latest
            latest_apriltag_data = apriltag_data
            
            # Update stats
            stats['apriltag_udp_events'] += 1
            stats['apriltag_events'] += 1
            stats['esp32_udp_active'] = True
            
            # Broadcast to GDevelop clients via WebSocket (async)
            broadcast_message = {
                'event': 'apriltag_data',
                **apriltag_data
            }
            
            # Schedule async broadcast (run in event loop)
            asyncio.create_task(broadcast_to_gdevelop(broadcast_message))
            
            print(f"*** Esp32 -->> SvHub .... .....: UDP packet | 📡 ID={apriltag_data.get('tag_id')}, Pos=({apriltag_data.get('x_cm', 0):.1f},{apriltag_data.get('y_cm', 0):.1f},{apriltag_data.get('z_cm', 0):.1f})cm | From={addr[0]}:{addr[1]} | GDevelop_clients={stats['gdevelop_clients']}")
            
            #### jwc 26-0131-0030 NEW: Send UDP acknowledgment back to ESP32
            # Create acknowledgment message (JSON)
            ack_message = {
                'event': 'apriltag_ack',
                'status': 'received',
                'tag_id': apriltag_data.get('tag_id'),
                'server_timestamp': time.time()
            }
            ack_json = json.dumps(ack_message).encode('utf-8')
            
            # Send UDP response back to ESP32
            self.transport.sendto(ack_json, addr)
            print(f"*** Esp32 <<-- SvHub .... .....: UDP ACK sent to {addr[0]}:{addr[1]}")
            
        except json.JSONDecodeError as e:
            print(f"❌ Invalid JSON in UDP packet from {addr[0]}: {e}")
            print(f"   Raw data: {data[:100]}")
        except Exception as e:
            print(f"❌ Error handling UDP packet from {addr[0]}: {e}")
    
    def error_received(self, exc):
        """Called when UDP error occurs"""
        print(f"❌ UDP error: {exc}")

# ============================================================================
# HTTP ENDPOINTS (Flask)
# ============================================================================

app = Flask(__name__)
CORS(app, origins="*", supports_credentials=True)

@app.route('/')
def home():
    """Status web page"""
    uptime_min = int((time.time() - stats['start_time']) / 60)
    
    html = f"""
    <!DOCTYPE html>
    <html>
    <head>
        <title>Async WebSocket Server</title>
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
                <h1>🚀 Async WebSocket Server (websockets library)</h1>
                <p>Reliable async WebSocket with proper timeout handling</p>
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
                <p><code>ws://localhost:{WEBSOCKET_PORT}/</code></p>
                <p><code>ws://[YOUR_IP]:{WEBSOCKET_PORT}/</code></p>
                
                <h3>📨 Message Format (JSON)</h3>
                <pre style="background: #2c3e50; padding: 10px; border-radius: 5px;">
// Identify as ESP32
{{"event": "identify", "data": {{"type": "esp32", "auth_token": "..."}}}}

// Send AprilTag data
{{"event": "apriltag_data", "tag_id": 5, "x_cm": 10.5, ...}}

// Identify as GDevelop
{{"event": "identify", "data": {{"type": "gdevelop"}}}}
                </pre>
                
                <h3>✅ Key Improvements</h3>
                <ul>
                    <li>✅ <strong>No more freeze!</strong> Async for loop with built-in timeout</li>
                    <li>✅ <strong>Proper error recovery</strong> - handles incomplete messages gracefully</li>
                    <li>✅ <strong>No ESP32 changes!</strong> Standard WebSocket protocol</li>
                    <li>✅ <strong>Production-ready</strong> - websockets library (11+ years, millions of users)</li>
                </ul>
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
        'protocol': 'websockets_async',
        'uptime_seconds': int(time.time() - stats['start_time']),
        'websocket': {
            'active_connections': stats['active_connections'],
            'esp32_connected': stats['esp32_connected'],
            'gdevelop_clients': stats['gdevelop_clients']
        }
    })

#### jwc 26-0128-1440 NEW: HTTP POST endpoint for ESP32 AprilTag data (stateless alternative to WebSocket)
@app.route('/apriltag', methods=['POST'])
def apriltag_http():
    """
    HTTP POST endpoint for AprilTag data from ESP32
    
    Alternative to WebSocket - stateless, simpler, no memory leaks!
    
    Expected JSON body:
    {
        "tag_id": 5,
        "decision_margin": 123.4,
        "yaw": 45.0,
        "pitch": 10.0,
        "roll": 5.0,
        "x_cm": 10.5,
        "y_cm": 20.3,
        "z_cm": 30.1,
        "range_cm": 35.2,
        "timestamp": 1234567890,
        "camera_name": "Waveshare-ESP32-S3",
        "smartcam_ip": "10.0.0.123"
    }
    """
    #### jwc 26-0129-0441 FIX: Declare global at function start (before any code)
    global latest_apriltag_data, websocket_loop
    
    try:
        # Validate auth token from header
        auth_header = request.headers.get('Authorization', '')
        if auth_header != AUTH_TOKEN:
            print(f"❌ HTTP POST /apriltag: AUTH FAILED (token={auth_header[:20]}...)")
            return jsonify({'error': 'Invalid auth_token'}), 401
        
        # Parse JSON body
        data = request.get_json()
        if not data:
            return jsonify({'error': 'No JSON body'}), 400
        
        # Add server timestamp
        data['server_timestamp'] = time.time()
        
        # Store as latest
        latest_apriltag_data = data
        
        # Broadcast to GDevelop clients via WebSocket (async)
        broadcast_message = {
            'event': 'apriltag_data',
            **data
        }
        
        # Schedule async broadcast (run in event loop)
        asyncio.run_coroutine_threadsafe(
            broadcast_to_gdevelop(broadcast_message),
            websocket_loop
        )
        
        stats['apriltag_events'] += 1
        
        print(f"*** SvHub <<-- Esp32: HTTP POST /apriltag | 📡 ID={data.get('tag_id')}, Pos=({data.get('x_cm', 0):.1f},{data.get('y_cm', 0):.1f},{data.get('z_cm', 0):.1f})cm | GDevelop_clients={stats['gdevelop_clients']}")
        
        return jsonify({'status': 'received', 'event': 'apriltag_ack'}), 200
        
    except Exception as e:
        print(f"❌ Error handling HTTP POST /apriltag: {e}")
        return jsonify({'error': str(e)}), 500

# ============================================================================
# MAIN
# ============================================================================

def print_startup_info():
    """Print startup information"""
    import socket
    
    hostname = socket.gethostname()
    local_ip = socket.gethostbyname(hostname)
    
    print("\n" + "="*70)
    print("🚀 Multi-Protocol Server Starting (HTTP + WebSocket + UDP)...")
    print("="*70)
    print(f"🖥️  Hostname: {hostname}")
    print(f"🌐 Local IP: {local_ip}")
    print("="*70)
    print("📡 ENDPOINTS:")
    print(f"   🔌 WebSocket: ws://{local_ip}:{WEBSOCKET_PORT}/")
    print(f"      └─ Real-time bidirectional (ESP32 ↔ GDevelop)")
    print(f"   🌐 HTTP: http://{local_ip}:{SERVER_PORT}/")
    print(f"      └─ Status dashboard, JSON API, POST /apriltag")
    print(f"   🔥 UDP: {local_ip}:{UDP_PORT}")
    print(f"      └─ Fire-and-forget (ZERO memory leaks!)")
    print("="*70)
    print("✅ ESP32 Can Use ANY Protocol:")
    print("   1️⃣  WebSocket (bidirectional, ack)")
    print("   2️⃣  HTTP POST (stateless, simple)")
    print("   3️⃣  UDP (fire-and-forget, ZERO leaks!) ⭐ RECOMMENDED")
    print("="*70)
    print("✅ Key Benefits:")
    print("   ✅ No more freeze! (async for loop with timeout)")
    print("   ✅ Proper error recovery (handles incomplete messages)")
    print("   ✅ UDP = ZERO memory leaks (stateless)")
    print("   ✅ Production-ready (websockets + asyncio)")
    print("="*70)
    print("✅ Server ready and listening!")
    print("="*70 + "\n")

async def start_websocket_server():
    """Start async WebSocket server"""
    async with websockets.serve(websocket_handler, SERVER_HOST, WEBSOCKET_PORT):
        print(f"✅ WebSocket server listening on {SERVER_HOST}:{WEBSOCKET_PORT}")
        await asyncio.Future()  # Run forever

def start_flask_server():
    """Start Flask HTTP server in separate thread"""
    log = logging.getLogger('werkzeug')
    log.setLevel(logging.ERROR)
    app.run(host=SERVER_HOST, port=SERVER_PORT, debug=False)

#### jwc 26-0129-0446 FIX: Python Syntax Error FINALLY FIXED!
# Successfully fixed the Python syntax error by wrapping the main code in a main() function.
# This is the proper Python pattern for handling global variables.
#
# 🔧 Root Cause:
# Python's global declaration has strict scoping rules:
# - Module-level code (under if __name__ == '__main__':) is NOT a function scope
# - global declarations only work inside functions
# - Declaring global at module level causes syntax error
#
# ✅ Solution:
# WRONG - global at module level (not in a function):
#   if __name__ == '__main__':
#       global websocket_loop  # ❌ SyntaxError!
#       websocket_loop = loop
#
# CORRECT - wrap in main() function:
#   def main():
#       global websocket_loop  # ✅ Works! (inside function)
#       websocket_loop = loop
#
#   if __name__ == '__main__':
#       main()

#### jwc 26-0130-1130 NEW: Start UDP server
async def start_udp_server():
    """Start async UDP server for ESP32 AprilTag data"""
    loop = asyncio.get_event_loop()
    transport, protocol = await loop.create_datagram_endpoint(
        lambda: UDPProtocol(),
        local_addr=(SERVER_HOST, UDP_PORT)
    )
    print(f"✅ UDP server listening on {SERVER_HOST}:{UDP_PORT}")
    return transport

def main():
    """Main entry point - sets up event loop and starts servers"""
    global websocket_loop
    
    print_startup_info()
    
    try:
        # Start Flask in background thread
        flask_thread = threading.Thread(target=start_flask_server, daemon=True)
        flask_thread.start()
        
        #### jwc 26-0128-1440 NEW: Store event loop for HTTP endpoint async broadcast
        # Get the event loop and store it globally so Flask HTTP endpoint can schedule async tasks
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        websocket_loop = loop
        
        #### jwc 26-0130-1130 NEW: Start UDP server alongside WebSocket
        # Run both UDP and WebSocket servers concurrently
        async def run_servers():
            # Start UDP server
            udp_transport = await start_udp_server()
            
            # Start WebSocket server (runs forever)
            await start_websocket_server()
        
        # Start both servers
        loop.run_until_complete(run_servers())
        
    except KeyboardInterrupt:
        print("\n🛑 Server stopped")
    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == '__main__':
    main()