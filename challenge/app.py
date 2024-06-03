from flask import render_template, Flask

import proto.robot_pb2 as robot_pb2
import proto.robot_pb2_grpc as robot_pb2_grpc
import grpc
import threading
import schedule
from google.protobuf.empty_pb2 import Empty 
import requests
from base64 import b64encode

# from os import sys
app = Flask(__name__)
app.robot_pose_data = {
    "x": 0.0,
    "y": 0.0,
    "yaw": 0.0
}  # Inicializa marker_data como un diccionario vacío
app.result_img = {
    "bytes": None
}

def call_api():
    global app
    url = "http://127.0.0.1:8042/api/v1/getrobotdata" # 
    try:
        response = requests.post(url, json={})
        response.raise_for_status()
        result = response.json()
        print(result)
    except requests.RequestException as e:
        print("Error haciendo la solicitud a la API:", e)


def call_grpc_api():
    global app
    try:
        print("Conectando al servidor gRPC en localhost:50051")
        channel = grpc.insecure_channel('192.168.3.168:50051')
        stub = robot_pb2_grpc.RobotServiceStub(channel)
        response = stub.GetRobotData(Empty())

        new_pose_data = {
            "x": response.pose.x,
            "y": response.pose.y,
            "yaw": response.pose.yaw
        }

        if new_pose_data:
            app.robot_pose_data = new_pose_data
            print("Datos del servidor gRPC actualizados correctamente.")


        # Obtener la imagen resultante
	
        data = response.image.image_bytes
        decoded_image = b64encode(data).decode()
        print(decoded_image[0:20])
        app.result_img["bytes"] = decoded_image  # Guardar la imagen en app.result_img
        print("Imagen del servidor gRPC obtenida correctamente.")
    except grpc.RpcError as e:
        print("Error llamando al servidor gRPC:", e)
    except Exception as e:
        print("Error inesperado:", e)


@app.route("/")
def hello_world():
    call_grpc_api()
    return render_template("index.html", pose_data=app.robot_pose_data, img_data=app.result_img)

@app.route('/api/robot_data')
def get_pose_data():
    return app.robot_pose_data

@app.route('/api/robot_img')
def get_image():
    global app
    if app.result_img is not None:
        return "data:image/jpg;base64, " + app.result_img["bytes"].replace('"','')
    else:
        return "data:image/jpeg;base64, /9j/4AAQSkZJRgABAQEASABIAAD/4QCCRXhpZgAATU0AKgAAAAgAAYdpAAQAAAABAAAAGgAAAAAABJADAAIAAAAUAAAAUJAEAAIAAAAUAAAAZJKRAAIAAAADMDAAAJKSAAIAAAADMDAAAAAAAAAyMDI0OjA1OjMxIDE5OjM2OjAyADIwMjQ6MDU6MzEgMTk6MzY6MDIAAAD/4QGcaHR0cDovL25zLmFkb2JlLmNvbS94YXAvMS4wLwA8P3hwYWNrZXQgYmVnaW49J++7vycgaWQ9J1c1TTBNcENlaGlIenJlU3pOVGN6a2M5ZCc/Pg0KPHg6eG1wbWV0YSB4bWxuczp4PSJhZG9iZTpuczptZXRhLyI+PHJkZjpSREYgeG1sbnM6cmRmPSJodHRwOi8vd3d3LnczLm9yZy8xOTk5LzAyLzIyLXJkZi1zeW50YXgtbnMjIj48cmRmOkRlc2NyaXB0aW9uIHJkZjphYm91dD0idXVpZDpmYWY1YmRkNS1iYTNkLTExZGEtYWQzMS1kMzNkNzUxODJmMWIiIHhtbG5zOnhtcD0iaHR0cDovL25zLmFkb2JlLmNvbS94YXAvMS4wLyI+PHhtcDpDcmVhdGVEYXRlPjIwMjQtMDUtMzFUMTk6MzY6MDI8L3htcDpDcmVhdGVEYXRlPjwvcmRmOkRlc2NyaXB0aW9uPjwvcmRmOlJERj48L3g6eG1wbWV0YT4NCjw/eHBhY2tldCBlbmQ9J3cnPz7/2wBDAAYEBQYFBAYGBQYHBwYIChAKCgkJChQODwwQFxQYGBcUFhYaHSUfGhsjHBYWICwgIyYnKSopGR8tMC0oMCUoKSj/2wBDAQcHBwoIChMKChMoGhYaKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCj/wAARCACWAMgDASIAAhEBAxEB/8QAHwAAAQUBAQEBAQEAAAAAAAAAAAECAwQFBgcICQoL/8QAtRAAAgEDAwIEAwUFBAQAAAF9AQIDAAQRBRIhMUEGE1FhByJxFDKBkaEII0KxwRVS0fAkM2JyggkKFhcYGRolJicoKSo0NTY3ODk6Q0RFRkdISUpTVFVWV1hZWmNkZWZnaGlqc3R1dnd4eXqDhIWGh4iJipKTlJWWl5iZmqKjpKWmp6ipqrKztLW2t7i5usLDxMXGx8jJytLT1NXW19jZ2uHi4+Tl5ufo6erx8vP09fb3+Pn6/8QAHwEAAwEBAQEBAQEBAQAAAAAAAAECAwQFBgcICQoL/8QAtREAAgECBAQDBAcFBAQAAQJ3AAECAxEEBSExBhJBUQdhcRMiMoEIFEKRobHBCSMzUvAVYnLRChYkNOEl8RcYGRomJygpKjU2Nzg5OkNERUZHSElKU1RVVldYWVpjZGVmZ2hpanN0dXZ3eHl6goOEhYaHiImKkpOUlZaXmJmaoqOkpaanqKmqsrO0tba3uLm6wsPExcbHyMnK0tPU1dbX2Nna4uPk5ebn6Onq8vP09fb3+Pn6/9oADAMBAAIRAxEAPwDwyiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKK2vCWkprGtRxXTmKwhVrm8mH/LOBBlyPcjgerEDvQAz/hG9Z/sX+1v7Nuf7Oxu8/Zxtzjd67c8bumeM1kV6X/xNRCPiO32P+x2vP7O/s7zG3eTt2+RjGNnl/L19+tcb4s0lNG1qSC2kM1jKq3FnMf8AlrA4yh+uOD6EEdqAMaiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAK6yU/wBk/Dy3+zg+brdw/nygcCKEjbF+LHefolcnWxoviXV9Ehkh028aOCRt7QuiyRlsY3bWBGcd8ZoAv/29L/wrYaB9pt/K/tX7Z9n8t/N/1W3dv+5s7beueelSof7W+HkxuAfO0S4RYJT/ABRTFsxfg67h/vPTf+E+8Rf8/Vr/AOAFv/8AEVm614k1fW4Y4dSvGlgjbekKoscYbpu2qAM++M0AY9FFFABRRRQAUUUUAFFFFABRRRQAUUUUAFFFFABRRRQAUUUUAFFFFAEtrBJdXMVvAu+aVxGi5xlicAVcOjagfM8q1knEe7e0A81V2gEklcjgHmqMTBJEcqrhSDtbOD7HHNdPD4vke1vor+1E/wBoXahSVk2fu9nJ5J4A5JyeQc7jQBjDRNU+0+Q+n3ccoxuEkLLtBOATkcDIIyfQ1FHpd/LGZIbO5liEnlb0iZl35A25AxnJHHuK1ovF2pRzSygW5lk3Zcocjc0jEdeRmVuue3pVOx169slKwmMqSxIZc53MjEfiY1/WgCCXSNRi8kSWF0rTOY41MRBZhjIAxnPIpG0nUVGTYXYG1nyYW+6v3j06DIye2asT67dTSW77IUNuT5W1T8q7FQL15ACjrz71JJ4jvZbOO2nCSxJCIQrs+MDbtIG7AI2LyAM45zQBUk0fUo8eZYXSsWCBTEwbJwBxjPJIA+tPg0PU5mcLY3KhITcMzRMoEYVm3ZI6YVseuK0J/F+qzTSyPIm6QPuzubluSRuJxhsMMYAIBAps3ivUJvOM3lyNKhVi5dsEq6lgC2ASJH4HHPSgDn6KfKweR3CKgYkhVzhfYZ7UygAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKAP/Z"


def main():
    #schedule.run_continuously()
    t = threading.Thread(target=call_grpc_api)
    t.start()

    threading.Timer(5,hello_world).start()

    try:
        app.run(debug=False)
    except KeyboardInterrupt:
        sys.exit(0)

if __name__ == "__main__":
    main()
