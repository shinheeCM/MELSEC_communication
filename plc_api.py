from flask import Flask, jsonify, request
import threading
import time
import pymcprotocol

app = Flask(__name__)

# PLC connection
pymc3e = pymcprotocol.Type3E()
pymc3e.connect("192.168.200.10", 3001)
print("Connected to PLC")

# Global signals
loop_ = False
d_values = {}

# Helpers
def read_register(name):
    val, _ = pymc3e.randomread(word_devices=[name], dword_devices=[])
    return val[0]

def write_register(name, value):
    pymc3e.randomwrite(word_devices=[name], word_values=[value], dword_devices=[], dword_values=[])

# PC -> PLC
# Background thread function to toggle D2000 every 3 seconds 
def toggle_D2000_signal():
    while True:
        # write_register("D2011", 0)
        # write_register("D2012", 0)
        write_register("D2000", 1)
        # print("D2000 set to 1")
        time.sleep(3)
        write_register("D2000", 0)
        # print("D2000 set to 0")
        time.sleep(3)

# Start the background thread
toggle_thread = threading.Thread(target=toggle_D2000_signal, daemon=True)
toggle_thread.start()

# Read cycle thread
def plc_read_cycle():
    global d_values, loop_
    while True:
        d_values = {
            "D2010": read_register("D2010"),
            "D2011": read_register("D2011"),
            "D2005": read_register("D2005"),
            "D2012": read_register("D2012"),
            "D2013": read_register("D2013"),
            "D2014": read_register("D2014"),
            "D2015": read_register("D2015"),
        }
        if d_values["D2011"] and not d_values["D2005"] and d_values["D2012"] and not d_values["D2014"]:
            loop_ = True
        time.sleep(1)

# Background thread
threading.Thread(target=plc_read_cycle, daemon=True).start()

# === Flask API Endpoints ===

@app.route("/status", methods=["GET"])
def status():
    return jsonify(d_values)


# After calling robot from MES then From AMR-->
# Write -->
# D2005 = 1 (AMR if object detected)
# D2005 = 0 (AMR if object not detected)

@app.route('/amr/object-detection-status-amr', methods=['POST'])
def set_product_detected():
    data = request.get_json()
    if not data or 'type' not in data:
        return jsonify({"error": "Missing 'type' in JSON body"}), 400

    amr_type = data['type'].lower()

    if amr_type == 'loading':
        write_register("D2005", 0)
        return jsonify({"message": "D2005 set to 0 (loading)"}), 200

    elif amr_type == 'unloading':
        write_register("D2005", 1)
        return jsonify({"message": "D2005 set to 1 (unloading)"}), 200

    else:
        return jsonify({"error": "Invalid 'type'. Must be 'loading' or 'unloading'."}), 400


@app.route('/amr/object-detection-status-plc', methods=['POST'])
def object_detection_status():
    try:
        data = request.get_json()
        if not data or 'type' not in data:
            return jsonify({"status": "error", "message": "Missing 'type' in request"}), 400

        # Common: D2011 must be 1 (AMR aligned)
        aligned = d_values.get("D2011", 0) == 1

        if data['type'] == 'loading':
            # Object present: D2011 == 1 and D2013 == 1
            detected = aligned and d_values.get("D2013", 0) == 1
            return jsonify({"object_detected": detected}), 200

        elif data['type'] == 'unloading':
            # No object: D2011 == 1 and D2012 == 1
            no_object = aligned and d_values.get("D2012", 0) == 1
            return jsonify({"no_object_detected": no_object}), 200

        else:
            return jsonify({"status": "error", "message": "Invalid type. Use 'loading' or 'unloading'."}), 400

    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500


@app.route('/amr/move', methods=['POST'])
def amr_arrived():
    try:
        data = request.get_json()
        if not data or 'type' not in data:
            return jsonify({"status": "error", "message": "Missing 'type' in request"}), 400

        if data['type'] == 'loading':
            write_register("D2003", 1)
            return jsonify({"status": "success", "message": "D2003 set to 1 (AMR arrived)"}), 200

        elif data['type'] == 'unloading':
            write_register("D2004", 1)
            return jsonify({"status": "success", "message": "D2004 set to 1"}), 200

        else:
            return jsonify({"status": "error", "message": "Invalid type. Use 'A' or 'B'."}), 400

    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500


@app.route('/amr/run-plc-conveyor', methods=['POST'])
def amr_align_conveyor():
    try:
        data = request.get_json()
        if not data or 'type' not in data:
            return jsonify({"status": "error", "message": "Missing 'type' in request"}), 400

        if data['type'] == 'loading':
            write_register("D2003", 0)
            write_register("D2001", 1)
            while d_values.get("D2014", 0) != 1:
                time.sleep(1)
            return jsonify({
                "status": "success",
                "message": "Loading: D2003 set to 0, D2001 set to 1"
            }), 200

        elif data['type'] == 'unloading':
            write_register("D2004", 0)
            write_register("D2002", 1)
            while d_values.get("D2015", 0) != 1:
                time.sleep(1)
            return jsonify({
                "status": "success",
                "message": "Unloading: D2004 set to 0, D2002 set to 1"
            }), 200

        else:
            return jsonify({"status": "error", "message": "Invalid type. Use 'loading' or 'unloading'."}), 400

    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500
    

@app.route('/amr/alignment/confirmed', methods=['GET'])
def check_alignment_confirmed():
    alignment_type = request.args.get("type")

    if alignment_type == "loading":
        confirmed = (d_values.get("D2014", 0) == 1)
    elif alignment_type == "unloading":
        confirmed = (d_values.get("D2015", 0) == 1)
    else:
        return jsonify({
            "status": "error",
            "message": "Missing or invalid type. Use 'loading' or 'unloading'."
        }), 400

    return jsonify({
        "alignment_confirmed": confirmed
    }), 200


@app.route('/amr/confirm-product', methods=['POST'])
def confirm_product():
    try:
        data = request.get_json()
        if not data or 'type' not in data:
            return jsonify({"status": "error", "message": "Missing 'type' in request"}), 400

        if data['type'] == 'loading':
            # Loading: write D2006 = 1 and wait for D2014 == 2
            write_register("D2006", 1)

            while d_values.get("D2014", 0) != 2:
                time.sleep(1)

            write_register("D2001", 0)
            write_register("D2006", 0)

            return jsonify({"status": "success", "product_confirmed": True}), 200

        elif data['type'] == 'unloading':
            # Unloading: write D2007 = 1 and wait for D2015 == 2
            write_register("D2007", 1)

            while d_values.get("D2015", 0) != 2:
                time.sleep(1)

            write_register("D2002", 0)
            write_register("D2007", 0)

            return jsonify({"status": "success", "product_confirmed": True}), 200

        else:
            return jsonify({"status": "error", "message": "Invalid type. Use 'loading' or 'unloading'."}), 400

    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500


@app.route("/start_input_conveyor_sim", methods=["POST"])
def start_input_conveyor():
    """Scenario 1: No object on AMR conveyor, use Input Conveyor A"""
    # Start movement
    write_register("D2003", 1)
    time.sleep(3)
    write_register("D2001", 1)  # A Conveyor Arrival
    write_register("D2003", 0)

    # Wait until PLC sends D2014 = 1
    while read_register("D2014") != 1:
        time.sleep(1)

    # Simulate object transfer
    write_register("D2006", 1)

    while read_register("D2014") != 2:
        time.sleep(1)

    write_register("D2001", 0)

    return jsonify({"message": "Input conveyor sequence completed"})

@app.route("/start_discharge_conveyor_sim", methods=["POST"])
def start_discharge_conveyor():
    """Scenario 2: Object detected on AMR conveyor, use Discharge Conveyor B"""
    write_register("D2004", 1)  # Start Discharge Conveyor
    time.sleep(3)
    write_register("D2002", 1)  # B Conveyor Arrival
    write_register("D2004", 0)

    # Wait for ready signal
    while read_register("D2015") != 1:
        time.sleep(1)

    # Simulate discharge
    write_register("D2007", 1)

    while read_register("D2015") != 2:
        time.sleep(1)

    write_register("D2002", 0)

    return jsonify({"message": "Discharge conveyor sequence completed"})

@app.route("/toggle_signal", methods=["POST"])
def toggle_signal():
    """Toggle D2000 and D2010 every 3 seconds."""
    def toggler():
        while True:
            for reg in ["D2000", "D2010"]:
                write_register(reg, 1)
                time.sleep(3)
                write_register(reg, 0)
                # time.sleep(3)
    threading.Thread(target=toggler, daemon=True).start()
    return jsonify({"message": "Started toggling D2000 and D2010"})

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
