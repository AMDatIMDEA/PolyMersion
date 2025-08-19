# PolyMersion - Two-Bathroom Version (with queuing system)

This version runs the experiment with **two simultaneous baths**, managed through a queuing system. It uses sockets to coordinate the robot's movements in parallel.

---

## Architecture

- `listener.py`: Server that manages queued client requests.
- `client.py`: Client that represents a bath experiment.
- Communication based on TCP/IP sockets.
- Ideal for experiments with multiple groups in parallel.

---

## Execution

1. Navigate to the directory:
```bash
cd PolyMersion/scripts_v2
```

2. Upload the code to Arduino:
```bash
cd ../../Arduino/PoliMersion
open PoliMersion.ino
```

3. Install the dependencies:
```bash
pip install -r requirements.txt
```

4. Run the `listener` (server):
```bash
python listener.py
```

5. In another terminal, run one or more `client.py` scripts:
```bash
python client.py --config config1.json --setup 1
```

---
