"""로컬 발표자료 뷰어 — http://localhost:8765
gen_slides.py 가 바뀌면 다시 돌려 슬라이드 HTML 을 갱신하고, 브라우저는 /version 을 1초마다 보고 새로고침한다.
/_blob/<id> 는 아티팩트 업로드 id → assets/ 로컬 파일로 바꿔 준다."""
import json, os, subprocess, sys, threading
from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler

S = os.path.dirname(os.path.abspath(__file__))
SL = os.path.join(S, 'deck2', 'project', 'slides')
GEN = os.path.join(S, 'gen_slides.py')
BLOB = {'4a881bbb14d5d0ad085ad33bedb7cbb2': 'Pretendard-Regular.otf', 'fe6eacfc052cba1d95bcdba2354a137c': 'Pretendard-Bold.otf',
        'e745e843afd7ee3c26b75f062f2d0818': 'logo_a.png', '0c729f8a2237f3a2d3c940363343d76a': 'logo_b.png',
        'aec3d4938bc0a7aa27c3e059ff582169': 'fig5_ipc_vs_dem.png', '653bc8a89bd63246e934d0fd8a2c15d7': 'fig6_cost_coupled_vs_prerun.png',
        'c11be2bdceb46341869e87ea8a7745ff': 'fig5_ab.png', 'a119220b272835e4baeaebf3c90c4740': 'fig6_ab.png',
        '626a6c7d4fcd3c6a215313203cfe784f': 'crush3d_tight_still.png', 'fda41a7f784e2612d79c7b55516e094f': 'crush3d.gif',
        '9cdeca776862c0e56bc37175c5873ffb': 'crush2d.gif', 'a223490e04c46501e5a26b25f35c64f8': 'fig_ipc_problem.png', 'e26fe63f8218394d461ade6b917a3c5c': 'fig_escape.png', '75a77b12dbe2cc4b373becf5f20ce350': 'cdm_impact_3d.mp4', 'e7cc80d354ce38c0d87acbd606c9f6f3': 'fig5_intuitive.png', 'f2b0a37c030284743c65b2d73b5f3c60': 'fig_ipc_problem_v2.png',
        '0358a6a30d03b1f0c3a6040c6e45e164': 'fig_ipc_problem.png',
        '9cf5453c8dd9c900b98e1e8f59d09266': 'crush_still.png', 'e3a5930830a09bd3236ab68e3b228488': 'cdm_still.png'}
lock, gen_mtime, gen_err = threading.Lock(), [0.0], ['']


def regen():
    """gen_slides.py 가 마지막 실행 이후 바뀌었으면 다시 돌린다. 에러는 뷰어 상단에 띄운다."""
    with lock:
        m = os.path.getmtime(GEN)
        if m <= gen_mtime[0]:
            return
        gen_mtime[0] = m
        r = subprocess.run([sys.executable, '-I', GEN], capture_output=True, text=True, encoding='utf-8', errors='replace')
        gen_err[0] = '' if r.returncode == 0 else r.stderr[-3000:]


def version():
    regen()
    fs = [os.path.join(SL, f) for f in os.listdir(SL)] + [os.path.join(S, 'deck2', 'project', 'deck.json'), os.path.join(S, 'viewer.html')]
    return max(os.path.getmtime(f) for f in fs)


class H(SimpleHTTPRequestHandler):
    def __init__(self, *a, **k): super().__init__(*a, directory=S, **k)

    def log_message(self, *a): pass

    def end_headers(self):
        self.send_header('Cache-Control', 'no-store')
        super().end_headers()

    def _send(self, body, ctype):
        b = body.encode('utf-8')
        self.send_response(200); self.send_header('Content-Type', ctype); self.send_header('Content-Length', str(len(b)))
        self.end_headers(); self.wfile.write(b)

    def do_GET(self):
        p = self.path.split('?')[0]
        if p == '/':
            self.path = '/viewer.html'
        elif p == '/version':
            return self._send(json.dumps({'v': version(), 'err': gen_err[0]}), 'application/json')
        elif p.startswith('/_blob/'):
            self.path = '/assets/' + BLOB.get(p[7:], 'missing')
        return super().do_GET()


if __name__ == '__main__':
    regen()
    print('http://localhost:8765', flush=True)
    ThreadingHTTPServer(('127.0.0.1', 8765), H).serve_forever()
