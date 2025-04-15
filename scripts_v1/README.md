# PolyMersion - Single-Bath Version

This script allows you to automatically run a polymer sample immersion experiment using a single bath, a UR robot, and an Arduino.

---

## Execution

1. Navigate to the directory:
```bash
cd PolyMersion/scripts_v1
```

2. Configure the experiment by editing the `config.json` file.

3. Upload the code to Arduino:
```bash
cd ../../Arduino/PoliMersion
open PoliMersion.ino # Or open it with the Arduino IDE
```

4. Install the dependencies:
```bash
pip install -r requirements.txt
```

5. Run the experiment:
```bash
python main.py
```
