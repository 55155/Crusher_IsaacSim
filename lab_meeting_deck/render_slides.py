import os, re, subprocess, sys
S = os.path.dirname(os.path.abspath(__file__))
A = os.path.join(S, 'assets').replace('\\', '/')
SL = os.path.join(S, 'deck2', 'project', 'slides')
OUT = os.path.join(S, 'render'); os.makedirs(OUT, exist_ok=True)
blob = {'4a881bbb14d5d0ad085ad33bedb7cbb2': 'Pretendard-Regular.otf', 'fe6eacfc052cba1d95bcdba2354a137c': 'Pretendard-Bold.otf',
        'e745e843afd7ee3c26b75f062f2d0818': 'logo_a.png', '0c729f8a2237f3a2d3c940363343d76a': 'logo_b.png',
        'aec3d4938bc0a7aa27c3e059ff582169': 'fig5_ipc_vs_dem.png', '653bc8a89bd63246e934d0fd8a2c15d7': 'fig6_cost_coupled_vs_prerun.png',
        'c11be2bdceb46341869e87ea8a7745ff': 'fig5_ab.png', 'a119220b272835e4baeaebf3c90c4740': 'fig6_ab.png',
        '626a6c7d4fcd3c6a215313203cfe784f': 'crush3d_tight_still.png', 'fda41a7f784e2612d79c7b55516e094f': 'crush3d.gif', '9cdeca776862c0e56bc37175c5873ffb': 'crush2d.gif',
        'a223490e04c46501e5a26b25f35c64f8': 'fig_ipc_problem.png',
        '9cf5453c8dd9c900b98e1e8f59d09266': 'crush_still.png', 'e3a5930830a09bd3236ab68e3b228488': 'cdm_still.png'}
head = f"""<!doctype html><meta charset="utf-8"><style>
@font-face{{font-family:'Pretendard';src:url(file:///{A}/Pretendard-Regular.otf)}}
@font-face{{font-family:'Pretendard Bold';src:url(file:///{A}/Pretendard-Bold.otf)}}
*{{margin:0;padding:0;box-sizing:border-box}} html,body{{width:1920px;height:1080px;overflow:hidden}}
section{{position:relative;width:1920px;height:1080px;overflow:hidden}} aside{{display:none}}
table{{border-collapse:collapse;width:100%}} th{{font-family:'Pretendard Bold';font-weight:400;border-bottom:2px solid #262626;padding:14px 12px}}
td{{border-bottom:1px solid #D9D9D9;padding:14px 12px;vertical-align:top}}
x-icon{{display:inline-block;background:currentColor}}
x-icon[name=ArrowRight]{{-webkit-mask:url("data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 24 24' fill='none' stroke='black' stroke-width='2' stroke-linecap='round' stroke-linejoin='round'%3E%3Cpath d='M5 12h14M12 5l7 7-7 7'/%3E%3C/svg%3E") center/contain no-repeat}}
x-icon[name=ArrowDown]{{-webkit-mask:url("data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 24 24' fill='none' stroke='black' stroke-width='2' stroke-linecap='round' stroke-linejoin='round'%3E%3Cpath d='M12 5v14M19 12l-7 7-7-7'/%3E%3C/svg%3E") center/contain no-repeat}}
</style>"""
edge = r'C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe'
for f in sys.argv[1:] or sorted(os.listdir(SL)):
    sid = f.replace('.html', '')
    h = open(os.path.join(SL, sid + '.html'), encoding='utf-8').read()
    h = re.sub(r'/_blob/([0-9a-f]{32})', lambda m: f'file:///{A}/{blob.get(m.group(1), "missing")}', h)
    p = os.path.join(OUT, sid + '_src.html')
    open(p, 'w', encoding='utf-8').write(head + h)
    subprocess.run([edge, '--headless=new', '--disable-gpu', '--allow-file-access-from-files', '--hide-scrollbars',
                    '--window-size=1920,1180', f'--screenshot={os.path.join(OUT, sid + ".png")}', 'file:///' + p.replace('\\', '/')],
                   capture_output=True, timeout=60)
    print(sid, os.path.exists(os.path.join(OUT, sid + '.png')))
