from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Request
from fastapi.responses import HTMLResponse, Response
from fastapi.templating import Jinja2Templates
import sqlite3
import json
from datetime import datetime, timezone
import asyncio
import csv
from io import StringIO
import time

app = FastAPI()
templates = Jinja2Templates(directory="templates")

# SQLite DB setup
conn = sqlite3.connect("power_data.db", check_same_thread=False)
c = conn.cursor()
c.execute('''CREATE TABLE IF NOT EXISTS readings
             (timestamp TEXT, total_power REAL,
              o1_power REAL, o1_voltage REAL, o1_current REAL, o1_state INTEGER,
              o2_power REAL, o2_voltage REAL, o2_current REAL, o2_state INTEGER)''')
conn.commit()

# In-memory state
latest_data = {
    "timestamp": datetime.now(timezone.utc).isoformat() + "Z",
    "total_power": 0.0,
    "outlet1": {"power": 0.0, "voltage": 0.0, "current": 0.0, "state": False},
    "outlet2": {"power": 0.0, "voltage": 0.0, "current": 0.0, "state": False}
}
websockets = []
desired_states = {"outlet1": True, "outlet2": True}  # Initial: both on
alert_threshold = 400.0  # Default watts
auto_control_enabled = False  # Default: manual only
alert_active = False
last_insert_time = 0

async def broadcast_data():
    broadcast_json = json.dumps(latest_data)
    for ws in websockets[:]:
        try:
            await ws.send_text(broadcast_json)
        except:
            websockets.remove(ws)

async def broadcast_alert():
    alert_data = json.dumps({"alert_active": alert_active, "threshold": alert_threshold})
    for ws in websockets[:]:
        try:
            await ws.send_text(alert_data)
        except:
            websockets.remove(ws)

def maybe_insert_to_db():
    global last_insert_time
    now = time.time()
    if now - last_insert_time > 1:  # Throttle to at most once per second
        c.execute('''INSERT INTO readings VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)''',
                  (latest_data['timestamp'], latest_data['total_power'],
                   latest_data['outlet1']['power'], latest_data['outlet1']['voltage'], latest_data['outlet1']['current'], int(latest_data['outlet1']['state']),
                   latest_data['outlet2']['power'], latest_data['outlet2']['voltage'], latest_data['outlet2']['current'], int(latest_data['outlet2']['state'])))
        conn.commit()
        last_insert_time = now

def build_command_byte():
    active_cmd = 1  # Always set to 1 to indicate a valid command to apply
    o1_state = 1 if desired_states['outlet1'] else 0
    o2_state = 1 if desired_states['outlet2'] else 0 # 00 - neither 01 - o1 - 10 - o2 - 11 both
    #value = active_cmd | (o1_state << 1) | (o2_state << 2)
    #value = 96 + o1_state + (2 * o2_state)
    value = '`' if not (o1_state or o2_state) else 'a' if (o1_state and not o2_state) else 'b' if (o2_state and not o1_state) else 'c'
    return bytes(value, 'utf-8')

@app.get("/", response_class=HTMLResponse)
async def root(request: Request):
    return templates.TemplateResponse("index.html", {"request": request})

# Individual POST endpoints for raw values
@app.post("/total_power")
async def update_total_power(request: Request):
    global alert_active
    try:
        body = await request.body()
        value = float(request.headers['Val'])
        latest_data['total_power'] = value
        latest_data['timestamp'] = datetime.now(timezone.utc).isoformat() + "Z"
        
        # Check alert and auto-control
        exceeded = latest_data['total_power'] > alert_threshold
        if exceeded != alert_active:
            alert_active = exceeded
            await broadcast_alert()
        if exceeded and auto_control_enabled:
            desired_states['outlet1'] = False
            desired_states['outlet2'] = False
        
        maybe_insert_to_db()
        await broadcast_data()
        return Response(content=None, status_code=200)
    except Exception as e:
        return Response(content=str(e), status_code=400)

@app.post("/voltage")
async def update_voltage(request: Request):
    try:
        body = await request.body()
        print(request.headers)
        print(request.query_params)
        print(body)
        value = float(request.headers['Val'])
        latest_data['outlet1']['voltage'] = value
        latest_data['outlet2']['voltage'] = value
        latest_data['timestamp'] = datetime.now(timezone.utc).isoformat() + "Z"
        maybe_insert_to_db()
        await broadcast_data()
        return Response(content=build_command_byte(), media_type="application/octet-stream", status_code=200)
    except Exception as e:
        return Response(content=str(e), status_code=400)

@app.post("/outlet1/power")
async def update_o1_power(request: Request):
    try:
        body = await request.body()
        value = float(body.decode().strip())
        latest_data['outlet1']['power'] = value
        latest_data['timestamp'] = datetime.now(timezone.utc).isoformat() + "Z"
        maybe_insert_to_db()
        await broadcast_data()
        return Response(content=build_command_byte(), media_type="application/octet-stream")
    except Exception as e:
        return Response(content=str(e), status_code=400)

@app.post("/outlet1/current")
async def update_o1_current(request: Request):
    try:
        body = await request.body()
        value = float(request.headers['val'])
        latest_data['outlet1']['current'] = value
        latest_data['timestamp'] = datetime.now(timezone.utc).isoformat() + "Z"
        maybe_insert_to_db()
        await broadcast_data()
        return Response(content=None, status_code=200)
    except Exception as e:
        return Response(content=str(e), status_code=400)

@app.post("/outlet1/state")
async def update_o1_state(request: Request):
    try:
        body = await request.body()
        value = int(request.headers['val'])
        latest_data['outlet1']['state'] = bool(value)
        latest_data['timestamp'] = datetime.now(timezone.utc).isoformat() + "Z"
        maybe_insert_to_db()
        await broadcast_data()
        return Response(content=None, status_code=200)
    except Exception as e:
        return Response(content=str(e), status_code=400)

@app.post("/outlet2/power")
async def update_o2_power(request: Request):
    try:
        body = await request.body()
        value = float(body.decode().strip())
        latest_data['outlet2']['power'] = value
        latest_data['timestamp'] = datetime.now(timezone.utc).isoformat() + "Z"
        maybe_insert_to_db()
        await broadcast_data()
        return Response(content=build_command_byte(), media_type="application/octet-stream")
    except Exception as e:
        return Response(content=str(e), status_code=400)

@app.post("/outlet2/current")
async def update_o2_current(request: Request):
    try:
        body = await request.body()
        value = float(request.headers['val'])
        latest_data['outlet2']['current'] = value
        latest_data['timestamp'] = datetime.now(timezone.utc).isoformat() + "Z"
        maybe_insert_to_db()
        await broadcast_data()
        return Response(content=None, status_code=200)
    except Exception as e:
        return Response(content=str(e), status_code=400)

@app.post("/outlet2/state")
async def update_o2_state(request: Request):
    try:
        body = await request.body()
        value = int(request.headers['Val'])
        latest_data['outlet2']['state'] = bool(value)
        latest_data['timestamp'] = datetime.now(timezone.utc).isoformat() + "Z"
        maybe_insert_to_db()
        await broadcast_data()
        return Response(content=None, status_code=200)
    except Exception as e:
        return Response(content=str(e), status_code=400)

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    websockets.append(websocket)
    # Send initial data and alert state
    await websocket.send_json(latest_data)
    await websocket.send_json({"alert_active": alert_active, "threshold": alert_threshold})
    try:
        while True:
            await asyncio.sleep(3600)  # Keep connection open; no need for receive if push-only
    except WebSocketDisconnect:
        websockets.remove(websocket)

@app.get("/history")
async def get_history(limit: int = 100):
    c.execute("SELECT * FROM readings ORDER BY timestamp DESC LIMIT ?", (limit,))
    rows = c.fetchall()
    history = []
    for r in rows:
        history.append({
            "timestamp": r[0],
            "total_power": r[1],
            "outlet1": {"power": r[2], "voltage": r[3], "current": r[4], "state": bool(r[5])},
            "outlet2": {"power": r[6], "voltage": r[7], "current": r[8], "state": bool(r[9])}
        })
    return history

@app.post("/toggle/{outlet}")
async def toggle_outlet(outlet: str):
    if outlet == "outlet1":
        desired_states["outlet1"] = not desired_states["outlet1"]
    elif outlet == "outlet2":
        desired_states["outlet2"] = not desired_states["outlet2"]
    # Broadcast updated states (optimistic update for GUI)
    latest_data["outlet1"]["state"] = desired_states["outlet1"]
    latest_data["outlet2"]["state"] = desired_states["outlet2"]
    await broadcast_data()
    return {"success": True}

@app.post("/set_threshold")
async def set_threshold(request: Request):
    global alert_threshold
    data = await request.json()
    alert_threshold = float(data["threshold"])
    await broadcast_alert()
    return {"success": True}

@app.post("/toggle_auto")
async def toggle_auto():
    global auto_control_enabled
    auto_control_enabled = not auto_control_enabled
    return {"enabled": auto_control_enabled}

@app.get("/export_csv")
async def export_csv():
    output = StringIO()
    writer = csv.writer(output)
    writer.writerow(["timestamp", "total_power", "o1_power", "o1_voltage", "o1_current", "o1_state",
                     "o2_power", "o2_voltage", "o2_current", "o2_state"])
    c.execute("SELECT * FROM readings")
    writer.writerows(c.fetchall())
    return Response(content=output.getvalue(), media_type="text/csv", headers={"Content-Disposition": "attachment; filename=usage_data.csv"})
