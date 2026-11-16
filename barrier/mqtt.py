"""선택: MQTT 로 상태 내보내기 (Jane 언리얼 클라이언트가 이미 구독하는 형식에 맞춤).

  topic/robot    float32 × 6 (관절각, deg)  ← Jane InterfaceScene.cpp 가 sizeof(float)*6 으로 읽어 프리뷰 로봇을 움직임
  topic/barrier  JSON {mode, alpha, tracking, p_cmd, p_des, d[6], active[6]}  ← 언리얼에서 박스/접촉 표시를 만들 때 쓸 것

실험 자체에는 필요 없다 (Quest 입력은 APK 가 ZMQ 로 보내고, 뷰어는 HTTP). 언리얼 쪽에 같이 보여 주고 싶을 때만
--mqtt <broker>[:port] 로 켠다. pip install paho-mqtt 필요. 관절각 단위(deg)가 언리얼 SetJointValue 와 맞는지는 확인 필요.
"""
import json
import struct

import numpy as np


class MqttPublisher:
    def __init__(self, broker, port=1883):
        import paho.mqtt.client as mqtt
        self.c = mqtt.Client()
        self.c.connect(broker, port, keepalive=10)
        self.c.loop_start()

    def joints(self, q_deg):
        q = np.asarray(q_deg, float).ravel()
        if np.isfinite(q).all():
            self.c.publish("topic/robot", struct.pack("<6f", *q), qos=0)

    def barrier(self, **kw):
        clean = {k: (np.asarray(v).tolist() if isinstance(v, np.ndarray) else v) for k, v in kw.items()}
        self.c.publish("topic/barrier", json.dumps(clean), qos=0)

    def close(self):
        self.c.loop_stop()
        self.c.disconnect()
