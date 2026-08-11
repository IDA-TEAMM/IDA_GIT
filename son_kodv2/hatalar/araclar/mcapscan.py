"""Hafif MCAP tarayıcı — payload OKUMADAN topic/zaman serisi çıkarır.

Bu bag'lerde chunk'lar SIKIŞTIRILMAMIŞ (compression='') ve her chunk'tan sonra
MessageIndex kayıtları var. MessageIndex, o chunk içindeki her mesajın
(log_time, offset) çiftini kanal başına taşır → chunk gövdesini hiç okumadan
tam zaman serisi elde edilir. Kanal/şema tanımları için yalnız ilk chunk'lar
açılır.

Ayrıca selektif okuma: küçük topic'lerin (String, Float32MultiArray) gerçek
payload'ı, chunk_data_start + offset ile doğrudan seek edilerek alınır.
"""
import struct, os, sys
from collections import defaultdict

U16 = struct.Struct('<H'); U32 = struct.Struct('<I'); U64 = struct.Struct('<Q')


def _str(buf, p):
    n = U32.unpack_from(buf, p)[0]
    return buf[p + 4:p + 4 + n].decode('utf-8', 'replace'), p + 4 + n


class Scan:
    def __init__(self, path):
        self.path = path
        self.channels = {}        # id -> (topic, schema_name)
        self.schemas = {}         # id -> name
        self.times = defaultdict(list)   # topic -> [log_time_ns]
        self.chunk_of = []        # (data_start, data_len, chunk_index) dosya ofsetleri
        self.truncated = False

    # ---- chunk içi kayıtları gez (kanal/şema toplamak için) ----
    def _parse_chunk_records(self, buf):
        p = 0; n = len(buf)
        while p + 9 <= n:
            op = buf[p]; ln = U64.unpack_from(buf, p + 1)[0]
            q = p + 9
            if q + ln > n:
                break
            if op == 3:      # Schema
                sid = U16.unpack_from(buf, q)[0]
                name, _ = _str(buf, q + 2)
                self.schemas[sid] = name
            elif op == 4:    # Channel
                cid = U16.unpack_from(buf, q)[0]
                sid = U16.unpack_from(buf, q + 2)[0]
                topic, _ = _str(buf, q + 4)
                self.channels[cid] = (topic, self.schemas.get(sid, '?'))
            p = q + ln

    def scan(self, max_chunks_for_channels=4):
        fh = open(self.path, 'rb')
        size = os.path.getsize(self.path)
        fh.seek(8)                                   # magic
        chunks_read = 0
        while True:
            h = fh.read(9)
            if len(h) < 9:
                self.truncated = True
                break
            op = h[0]; ln = U64.unpack_from(h, 1)[0]
            body = fh.tell()
            if body + ln > size:                     # yarıda kesilmiş dosya
                self.truncated = True
                break
            if op == 6:                              # Chunk
                head = fh.read(40)
                if len(head) < 40:
                    self.truncated = True; break
                comp_len = U32.unpack_from(head, 28)[0]
                if comp_len != 0:                    # sıkıştırılmışsa bu yol geçersiz
                    raise RuntimeError('chunk sıkıştırılmış: bu tarayıcı desteklemiyor')
                rec_len = U64.unpack_from(head, 32)[0]
                data_start = body + 40
                self.chunk_of.append((data_start, rec_len))
                if chunks_read < max_chunks_for_channels:
                    fh.seek(data_start)
                    self._parse_chunk_records(fh.read(rec_len))
                    chunks_read += 1
                fh.seek(body + ln)                   # chunk gövdesini ATLA
            elif op == 7:                            # MessageIndex — asıl kaynak
                buf = fh.read(ln)
                cid = U16.unpack_from(buf, 0)[0]
                alen = U32.unpack_from(buf, 2)[0]
                p = 6; end = 6 + alen
                t = self.times[cid]
                while p + 16 <= end:
                    t.append(U64.unpack_from(buf, p)[0])
                    p += 16
            elif op == 3:
                buf = fh.read(ln); sid = U16.unpack_from(buf, 0)[0]
                name, _ = _str(buf, 2); self.schemas[sid] = name
            elif op == 4:
                buf = fh.read(ln); cid = U16.unpack_from(buf, 0)[0]
                sid = U16.unpack_from(buf, 2)[0]
                topic, _ = _str(buf, 4)
                self.channels[cid] = (topic, self.schemas.get(sid, '?'))
            elif op == 5:                            # chunk'sız düz Message
                buf = fh.read(ln)
                cid = U16.unpack_from(buf, 0)[0]
                self.times[cid].append(U64.unpack_from(buf, 6)[0])
            else:
                fh.seek(body + ln)
        # Kanal tanımları sonraki chunk'larda olabilir (ör. /livox/lidar ilk
        # chunk'ları tek başına doldurunca diğer topic'ler geç tanımlanır).
        # MessageIndex'te görülüp henüz çözülmemiş id kalmışsa chunk'ları
        # sırayla açıp hepsi çözülene kadar devam et.
        eksik = set(self.times) - set(self.channels)
        if eksik:
            for data_start, rec_len in self.chunk_of:
                fh.seek(data_start)
                self._parse_chunk_records(fh.read(rec_len))
                eksik = set(self.times) - set(self.channels)
                if not eksik:
                    break
        fh.close()
        # kanal id -> topic adına çevir
        out = {}
        for cid, ts in self.times.items():
            topic, typ = self.channels.get(cid, (f'<bilinmeyen id={cid}>', '?'))
            ts.sort()
            out[topic] = (typ, ts)
        return out

    # ---- selektif payload okuma ----
    def read_messages(self, topic, limit=None):
        """Bir topic'in ham CDR payload'larını (log_time, bytes) olarak döndür."""
        cid = next((c for c, (t, _) in self.channels.items() if t == topic), None)
        if cid is None:
            return []
        fh = open(self.path, 'rb'); size = os.path.getsize(self.path)
        fh.seek(8)
        out = []
        while True:
            h = fh.read(9)
            if len(h) < 9:
                break
            op = h[0]; ln = U64.unpack_from(h, 1)[0]
            body = fh.tell()
            if body + ln > size:
                break
            if op == 6:
                head = fh.read(40)
                if len(head) < 40:
                    break
                rec_len = U64.unpack_from(head, 32)[0]
                data_start = body + 40
                fh.seek(data_start)
                buf = fh.read(rec_len)
                p = 0
                while p + 9 <= len(buf):
                    o = buf[p]; l = U64.unpack_from(buf, p + 1)[0]
                    q = p + 9
                    if q + l > len(buf):
                        break
                    if o == 5 and U16.unpack_from(buf, q)[0] == cid:
                        out.append((U64.unpack_from(buf, q + 6)[0], buf[q + 22:q + l]))
                        if limit and len(out) >= limit:
                            fh.close(); return out
                    p = q + l
                fh.seek(body + ln)
            else:
                fh.seek(body + ln)
        fh.close()
        return out
