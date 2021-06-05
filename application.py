# -*- coding: utf-8 -*-
"""
Created on Thu May 27 20:09:52 2021

@author: janusz
"""

import json
import random
import time
import subprocess
from datetime import datetime

from flask import Flask, Response, render_template

TIMER_BASED=0
OWN_DRIVER=1

application = Flask(__name__)
random.seed()  # Initialize the random number generator


@application.route("/")
def index():
    return render_template("index.html")


def generate_random_data(sensor_number=TIMER_BASED):
    while True:
        u = subprocess.check_output(['python', 'serial_read.py']).decode('utf-8').strip()
        u = u.split()
        value_timer_based = float(u[TIMER_BASED])
        value_own_driver = float(u[OWN_DRIVER])
        json_data = json.dumps(
            {
                "time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                "value_timer_based": value_timer_based,
                "value_own_driver": value_own_driver,
            }
        )
        yield f"data:{json_data}\n\n"
        time.sleep(1)


@application.route("/chart-data")
def chart_data():
    return Response(generate_random_data(), mimetype="text/event-stream")


if __name__ == "__main__":
    application.run(host="0.0.0.0", debug=True, threaded=True)