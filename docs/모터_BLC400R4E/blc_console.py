"""BLC-400R4E RS-485 콘솔 — 판매처 아두이노 예제(V1.2)의 PC 판

아두이노 대신 PC + USB-RS485 컨버터를 쓸 뿐, 프레임·명령·로그 형식은 예제와 동일하다.
  아두이노 SoftwareSerial  -> PC COM 포트
  MAX485 모듈 + DE/RE(D3)  -> USB-RS485 컨버터(방향 전환 자동)
  sendFrame()              -> frame() / send()

프레임 구조  FF FE | ID | Data크기 | Checksum | Mode | Data...
  Checksum = ~(ID + Data크기 + Mode + Data바이트 합) 의 하위 1바이트

명령 (예제와 동일)
  e / E   : 제어 On / Off
  f1000   : CW  100.0RPM (0.1RPM 단위, 도달시간 1.0s)
  r500    : CCW  50.0RPM
  s       : 감속 정지
  g9000   : CW  90.00도 위치 이동 (0.01도 단위)
  G9000   : CCW 90.00도
  c       : 분해능 등록 (1000CPR -> 250, CPR/4 규칙)
  d212    : 감속비 등록 (1/212 -> 2120, 0.1비 단위)
  z       : 위치 0 초기화
  p / v   : 위치 / 속도 피드백 요청

사용법
  python blc_console.py                 대화형
  python blc_console.py --script a.txt  파일의 명령을 순서대로 실행
  python blc_console.py --log out.txt   TX/RX 로그를 파일로도 저장
  옵션: --port COM5  --baud 9600  --id 0
"""

import argparse
import sys
import time

import serial

ENC_PPR = 250          # 1000CPR / 4 (프로토콜 규칙: CPR 은 4로 나눠 입력)

_log_fp = None


def out(line=""):
    print(line)
    if _log_fp:
        _log_fp.write(line + "\n")
        _log_fp.flush()


def hx(b):
    return " ".join(f"{x:02X}" for x in b)


def frame(mid, mode, data=b""):
    body = bytes([mode]) + data
    size = len(body) + 1                       # Mode + Data + Checksum
    chk = (~(mid + size + sum(body))) & 0xFF
    return bytes([0xFF, 0xFE, mid, size, chk]) + body


def send(ser, mid, mode, data=b"", note=""):
    """프레임 송신 후 응답을 프레임 단위로 기다린다(고정 sleep 아님)."""
    pkt = frame(mid, mode, data)
    ser.reset_input_buffer()
    ser.write(pkt)
    ser.flush()

    buf = bytearray()
    t0 = time.monotonic()
    while time.monotonic() - t0 < 0.4:
        n = ser.in_waiting
        if n:
            buf += ser.read(n)
        b = bytes(buf)
        if b.startswith(pkt):                  # 컨버터 에코 제거
            b = b[len(pkt):]
        if len(b) >= 4 and b[0] == 0xFF and len(b) >= 4 + b[3]:
            break
        time.sleep(0.001)

    out(f"TX: {hx(pkt)}" + (f"    {note}" if note else ""))
    out(f"RX: {hx(buf) if buf else '(응답 없음)'}")
    decode(bytes(buf), pkt)
    return bytes(buf)


def decode(buf, sent):
    """0xD1 / 0xD2 피드백만 사람이 읽을 수 있게 풀어 준다."""
    b = buf[len(sent):] if buf.startswith(sent) else buf
    if len(b) < 6 or b[0] != 0xFF or b[1] != 0xFE or len(b) < 4 + b[3]:
        return
    f = b[:4 + b[3]]
    if ((~(f[2] + f[3] + sum(f[5:]))) & 0xFF) != f[4]:
        out("    [체크섬 불일치]")
        return
    mode, d = f[5], f[6:]
    u16 = lambda i: (d[i] << 8) | d[i + 1]
    if mode == 0xD1 and len(d) >= 6:
        out(f"    위치: {'CW' if d[0] else 'CCW'} {u16(1)/100:.2f}도, "
            f"속도 {u16(3)/10:.1f}RPM, 전류 {d[5]*0.1:.1f}A")
    elif mode == 0xD2 and len(d) >= 6:
        out(f"    속도: {'CW' if d[0] else 'CCW'} {u16(1)/10:.1f}RPM, "
            f"위치 {u16(3)/10:.1f}도, 전류 {d[5]*0.1:.1f}A")


def handle(ser, mid, cmd):
    cmd = cmd.strip()
    if not cmd or cmd.startswith("#"):
        if cmd.startswith("#"):
            out(cmd)
        return True
    c, arg = cmd[0], cmd[1:].strip()
    try:
        v = int(arg) if arg else 0
    except ValueError:
        v = 0
    u16b = lambda x: bytes([(x >> 8) & 0xFF, x & 0xFF])
    clamp = lambda x, lo, hi: max(lo, min(hi, x))

    if c == "e":
        send(ser, mid, 0x0C, bytes([0x00]), "제어 On")
    elif c == "E":
        send(ser, mid, 0x0C, bytes([0x01]), "제어 Off")
    elif c == "f":
        send(ser, mid, 0x03, bytes([0x01]) + u16b(clamp(v, 1, 65533)) + bytes([10]),
             f"가감속 속도제어 CW {v/10:.1f}RPM / 도달 1.0s")
    elif c == "r":
        send(ser, mid, 0x03, bytes([0x00]) + u16b(clamp(v, 1, 65533)) + bytes([10]),
             f"가감속 속도제어 CCW {v/10:.1f}RPM / 도달 1.0s")
    elif c == "s":
        send(ser, mid, 0x03, bytes([0x01]) + u16b(1) + bytes([5]), "감속 정지")
        time.sleep(0.6)
        send(ser, mid, 0x03, bytes([0x01]) + u16b(1) + bytes([1]), "감속 정지(2)")
    elif c == "g":
        send(ser, mid, 0x02, bytes([0x01]) + u16b(clamp(v, 0, 65533)) + bytes([10]),
             f"가감속 위치제어 CW {v/100:.2f}도 / 도달 1.0s")
    elif c == "G":
        send(ser, mid, 0x02, bytes([0x00]) + u16b(clamp(v, 0, 65533)) + bytes([10]),
             f"가감속 위치제어 CCW {v/100:.2f}도 / 도달 1.0s")
    elif c == "c":
        send(ser, mid, 0x0A, u16b(ENC_PPR), f"분해능 등록 {ENC_PPR} (1000CPR / 4)")
    elif c == "d":
        if v < 1:
            out("감속비를 입력하십시오 (예 d212)")
            return True
        send(ser, mid, 0x0B, u16b(v * 10), f"감속비 1/{v} 등록 (전송값 {v*10})")
    elif c == "z":
        send(ser, mid, 0x0F, b"", "위치 0 초기화")
    elif c == "p":
        send(ser, mid, 0xA1, b"", "위치 피드백 요청")
    elif c == "v":
        send(ser, mid, 0xA2, b"", "속도 피드백 요청")
    elif c == "V":
        send(ser, mid, 0xCD, b"", "펌웨어 버전 요청")
    elif c == "w":
        time.sleep(v / 1000.0 if v else 0.5)
        out(f"(대기 {v if v else 500}ms)")
    elif c in ("q", "Q"):
        return False
    else:
        out("? e/E c d212 f300 r300 s g9000 G9000 z p v V w500 q")
    return True


def main():
    global _log_fp
    ap = argparse.ArgumentParser(description="BLC-400R4E RS-485 콘솔")
    ap.add_argument("--port", default="COM5")
    ap.add_argument("--baud", type=int, default=9600)
    ap.add_argument("--id", type=int, default=0)
    ap.add_argument("--script")
    ap.add_argument("--log")
    a = ap.parse_args()

    if a.log:
        _log_fp = open(a.log, "w", encoding="utf-8")

    ser = serial.Serial(a.port, a.baud, bytesize=8, parity="N", stopbits=1, timeout=0.05)
    out(f"BLC-400R4E ready — {a.port} {a.baud}bps 8N1, ID 0x{a.id:02X}")
    out("명령: e/E c d212 f300 r300 s g9000 G9000 z p v V w500 q")
    out("")
    try:
        if a.script:
            for line in open(a.script, encoding="utf-8"):
                if not handle(ser, a.id, line):
                    break
        else:
            for line in sys.stdin:
                if not handle(ser, a.id, line):
                    break
    finally:
        send(ser, a.id, 0x0C, bytes([0x01]), "제어 Off (종료)")
        ser.close()
        if _log_fp:
            _log_fp.close()


if __name__ == "__main__":
    main()
