from flask import Flask, render_template, request, redirect
import datetime

app = Flask(__name__)

LOG_FILE = "received_commands.txt"

@app.route("/", methods=["GET", "POST"])
def index():
    message = ""
    if request.method == "POST":
        alpha = request.form.get("alpha")
        theta = request.form.get("theta")
        if alpha and theta:
            command = f"alpha={alpha},theta={theta}"
            print(f"Received command: {command}")
            with open(LOG_FILE, "a") as f:
                timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                f.write(f"{timestamp} - Command: {command}\n")
            message = f"Sent → α = {alpha}°, θ = {theta}°"
    return render_template("index.html", message=message)

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)

