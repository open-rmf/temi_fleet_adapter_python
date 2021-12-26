from aiohttp import web
import socketio
import argparse


sio = socketio.AsyncServer(cors_allowed_origins="*")
app = web.Application()
sio.attach(app)


@sio.event
def connect(sid, headers, environ):
    try:
        robot_name = headers["HTTP_ROBOT_NAME"]
        print(f"connect {sid} to room {robot_name}")
        sio.enter_room(sid, robot_name)
    except Exception as e:
        print(e)


@sio.event
def disconnect(sid):
    print('disconnect ', sid)


@sio.event
async def robot_state(sid, data):
    for room in sio.rooms(sid):
        await sio.emit('robot_state', {'data': data}, room=room)
    print('robot_state: ', data)
    

@sio.event
async def battery_status(sid, data):
    for room in sio.rooms(sid):
        await sio.emit('battery_status', {'data': data}, room=room)
    print('battery_status: ', data)


@sio.event
async def goToPosition(sid, data):
    for room in sio.rooms(sid):
        await sio.emit('goToPosition', {'data': data}, room=room)
    print('goToPosition: ', data)


@sio.event
async def tiltBy(sid, data):
    for room in sio.rooms(sid):
        await sio.emit('tiltBy', {'data': data}, room=room)
    print('tiltBy: ', data)


@sio.event
async def turnBy(sid, data):
    for room in sio.rooms(sid):
        await sio.emit('turnBy', {'data': data}, room=room)
    print('turnBy: ', data)


@sio.event
async def skidJoy(sid, data):
    for room in sio.rooms(sid):
        await sio.emit('skidJoy', {'data': data}, room=room)
    print('skidJoy: ', data)


@sio.event
async def stopMovement(sid, data):
    for room in sio.rooms(sid):
        await sio.emit('stopMovement', {'data': data}, room=room)
    print('stopMovement: ', data)


@sio.event
async def telepresence(sid, data):
    for room in sio.rooms(sid):
        await sio.emit('telepresence', {'data': data}, room=room)
    print('telepresence: ', data)

@sio.event
async def telepresenceEnd(sid, data):
    for room in sio.rooms(sid):
        await sio.emit('telepresenceEnd', {'data': data}, room=room)
    print('telepresenceEnd: ', data)


@sio.event
async def goTo(sid, data):
    for room in sio.rooms(sid):
        await sio.emit('goTo', {'data': data}, room=room)
    print('goTo: ', data)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, required=False,
                        help="Port to serve on", default=8008)
    args = parser.parse_args()
    web.run_app(app, host="0.0.0.0", port=args.port)
