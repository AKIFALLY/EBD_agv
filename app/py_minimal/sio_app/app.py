from fastapi import FastAPI, Request
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
import socketio

# 建立 Socket.IO ASGI App
sio = socketio.AsyncServer(async_mode="asgi")
sio_app = socketio.ASGIApp(sio)

# 建立 FastAPI App
app = FastAPI()
app.mount("/socket.io", sio_app)  # 將 socket.io 掛載到 FastAPI 路由下

# Jinja2 模板支援
templates = Jinja2Templates(directory="templates")


@app.get("/", response_class=HTMLResponse)
async def get_index(request: Request):
    return templates.TemplateResponse("index.html", {"request": request})

# Socket.IO 事件


@sio.event
async def connect(sid, environ):
    print("使用者連線:", sid)
    await sio.emit("server_message", "歡迎連線！", to=sid)


@sio.event
async def chat_message(sid, data):
    print("收到訊息:", data)
    await sio.emit("chat_message", f"伺服器收到：{data}")


@sio.event
async def disconnect(sid):
    print("使用者離線:", sid)
