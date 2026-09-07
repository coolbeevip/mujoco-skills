"""场景级观测记忆：记录看见的事实，不读取场景物体真值。"""

import json
import sqlite3
import uuid
from datetime import datetime, timezone


class ObjectMemory:
    def __init__(self, path=None):
        if path is not None:
            path.parent.mkdir(parents=True, exist_ok=True)
        self.db = sqlite3.connect(str(path) if path is not None else ":memory:")
        self.db.execute("""CREATE TABLE IF NOT EXISTS observations (
            id INTEGER PRIMARY KEY, session TEXT NOT NULL, scene TEXT NOT NULL,
            object_id TEXT NOT NULL, wall_time TEXT NOT NULL, data TEXT NOT NULL
        )""")
        self.db.execute(
            "CREATE INDEX IF NOT EXISTS memory_session ON observations(session, object_id, id)"
        )
        self.db.execute("""CREATE TABLE IF NOT EXISTS latest_objects (
            session TEXT NOT NULL, scene TEXT NOT NULL, object_id TEXT NOT NULL,
            wall_time TEXT NOT NULL, data TEXT NOT NULL, PRIMARY KEY(session, object_id)
        )""")
        # 最新摘要与完整历史分开索引；预览刷新不扫描所有历史观测帧。
        self.db.execute("""INSERT OR IGNORE INTO latest_objects
            SELECT session, scene, object_id, wall_time, data FROM observations
            WHERE id IN (SELECT MAX(id) FROM observations GROUP BY session, object_id)""")
        self.db.commit()
        self.session, self.scene = None, None
        self.objects = {}

    def begin_scene(self, scene):
        # 重置/重启会重建世界；以前的位置永久保留，但不能迁移为新世界的真值。
        self.session, self.scene = uuid.uuid4().hex, scene
        self.objects = {}

    def observe(self, detections, sim_time, observer):
        now = datetime.now(timezone.utc).isoformat()
        seen = set()
        for detection in detections:
            key = detection["object_id"]
            seen.add(key)
            old = self.objects.get(key, {})
            record = {
                **old,
                **detection,
                "first_seen": old.get("first_seen", now),
                "last_seen": now,
                "observed_at_s": sim_time,
                "observed_from": list(observer),
                "currently_visible": True,
            }
            # 仅见颜色、没有有效深度时，保留最后一次验证的位置和定位时间。
            if detection.get("position") is None and old.get("position") is not None:
                record["position"] = old["position"]
                record["name"] = old.get("name", record.get("name"))
                record["kind"] = old.get("kind", record.get("kind"))
                record["position_observed_at_s"] = old["position_observed_at_s"]
                record["position_verified_now"] = False
            else:
                record["position_verified_now"] = detection.get("position") is not None
                if record["position_verified_now"]:
                    record["position_observed_at_s"] = sim_time
            self.objects[key] = record
            self.db.execute(
                "INSERT INTO observations(session,scene,object_id,wall_time,data) VALUES (?,?,?,?,?)",
                (
                    self.session,
                    self.scene,
                    key,
                    now,
                    json.dumps(record, ensure_ascii=False, allow_nan=False),
                ),
            )
            self.db.execute(
                "INSERT OR REPLACE INTO latest_objects VALUES (?,?,?,?,?)",
                (
                    self.session,
                    self.scene,
                    key,
                    now,
                    json.dumps(record, ensure_ascii=False, allow_nan=False),
                ),
            )
        self.db.commit()
        for key, record in self.objects.items():
            if key not in seen:
                record["currently_visible"] = False
                record["position_verified_now"] = False

    def current(self):
        return [dict(record) for record in self.objects.values()]

    def target(self, color):
        record = self.objects.get(f"ball:{color}")
        if record is None or record.get("position") is None:
            return None
        return {**record, "observed_at_s": record["position_observed_at_s"]}

    def archive(self, limit=200):
        # 展示每个世界里每个物体的最后观测；完整轨迹仍保存在 observations 表。
        rows = self.db.execute(
            """SELECT session, scene, data FROM latest_objects ORDER BY wall_time DESC LIMIT ?""",
            (limit,),
        ).fetchall()
        records = []
        for session, scene, data in rows:
            record = json.loads(data)
            historical = session != self.session
            if historical:
                record.update(currently_visible=False, position_verified_now=False)
            else:
                record.update(self.objects.get(record["object_id"], {}))
            records.append(
                {**record, "session": session, "scene": scene, "historical": historical}
            )
        return records

    def close(self):
        self.db.close()
