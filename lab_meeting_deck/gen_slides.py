import os
S = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(S, 'deck2', 'project', 'slides')
BF = "font-family:'Pretendard Bold', Arial, sans-serif;font-weight:400"
# 강조는 빨간 박스 대신 '제목이 테두리에 걸린 틀'(사용자 스타일, 2026-11-02 선택) — 색은 무채색만
RED, RBG, RBD = '#262626', '#FFFFFF', '#7F7F7F'

def B(t, c='#262626'): return f'<b style="{BF};color:{c}">{t}</b>'
def R(t): return B(t, RED)

def sec(sid, label, title, body, note):
    return f'''<section id="{sid}" data-transition="fade" style="background:#FFFFFF;color:#404040;font-family:'Pretendard', Arial, sans-serif;font-weight:400">
<p style="position:absolute;left:57px;top:40px;font-size:30px;color:#404040">{label}</p>
<h2 style="position:absolute;left:78px;top:110px;font-size:48px;line-height:1.2;{BF};color:#505354">{title}</h2>
<div style="position:absolute;left:42px;top:200px;width:1836px;height:2px;background:#262626"></div>
<div style="position:absolute;left:78px;top:236px;width:1764px;height:800px;display:flex;flex-direction:column;justify-content:center;gap:32px">
{body}
</div>
<aside>{note}</aside>
</section>
'''

def bul(t, fs=32): return f'<p style="font-size:{fs}px;line-height:1.55;padding-left:32px;text-indent:-32px">•&nbsp; {t}</p>'
def head(t, fs=40): return f'<p style="font-size:{fs}px;{BF};color:#262626">{t}</p>'
def gray(t): return f'<p style="font-size:26px;color:#7F7F7F">{t}</p>'
def col(*xs, w='flex:1'): return f'<div style="{w};display:flex;flex-direction:column;gap:18px">' + ''.join(xs) + '</div>'
def row(*xs, gap=56, align='stretch', grow=False):
    g = 'flex:1;min-height:0;' if grow else ''
    return f'<div style="{g}display:flex;gap:{gap}px;align-items:{align}">' + ''.join(xs) + '</div>'
# 텍스트 위주 슬라이드는 카드가 남는 높이를 채운다 (row(grow=True) 안에서 늘어남)
def card(*xs, center=False, w='flex:1'):
    j = 'justify-content:center;' if center else ''
    return f'<div style="{w};display:flex;flex-direction:column;{j}gap:22px;border:1px solid #D9D9D9;padding:40px 44px">' + ''.join(xs) + '</div>'
BOTTOM = '<div style="margin-top:auto"></div>'
VR = '<div style="width:1px;background:#D9D9D9"></div>'
def key(label, t, fs=30):
    return frame(label, f'<p style="font-size:{fs}px;text-align:center">{t}</p>', h='padding:36px 30px 20px;')
def tag(t): return f'<p style="font-size:28px;color:#595959">→ {t}</p>'
def gbox(t, red=False, fs=30):
    # 강조 박스(red=True)는 색 대신 굵은 검정 테두리로 구분
    bd = '2px solid #262626' if red else '1px solid #BFBFBF'
    bg = '#FFFFFF' if red else '#F2F2F2'
    return f'<p style="font-size:{fs}px;{BF};color:#262626;background:{bg};border:{bd};padding:22px 28px;text-align:center;line-height:1.35">{t}</p>'
ARR = '<x-icon name="ArrowRight" style="color:#595959;width:36px;height:36px;flex:none"></x-icon>'
ARD = '<x-icon name="ArrowDown" style="color:#595959;width:30px;height:30px;align-self:center"></x-icon>'
def flow(*boxes): return '<div style="display:flex;gap:16px;align-items:center;justify-content:center">' + ARR.join(boxes) + '</div>'
def frame(title, inner, w='width:100%', h=''):
    return (f'<div style="position:relative;margin-top:14px;{w};{h}border:1px solid #7F7F7F;padding:40px 28px 20px;display:flex;flex-direction:column;align-items:center;gap:12px">'
            f'<p style="position:absolute;top:-20px;left:50%;transform:translateX(-50%);background:#FFFFFF;padding:0 24px;font-size:28px;{BF};color:#404040;white-space:nowrap">{title}</p>'
            f'{inner}</div>')
def img(src, alt, w, h, extra='', video=None):
    v = f' data-video="{video}"' if video else ''
    return f'<img src="{src}"{v} alt="{alt}" style="width:{w}px;height:{h}px;object-fit:contain{extra}">'
def cap(t): return f'<p style="font-size:24px;color:#595959;text-align:center">{t}</p>'
def stat(num, t, red=True, fs=132, ts=32):
    c, bg, bd = (RED, RBG, RBD) if red else ('#404040', '#F2F2F2', '#BFBFBF')
    return (f'<div style="flex:1;display:flex;flex-direction:column;justify-content:center;gap:16px;background:{bg};border:1px solid {bd};padding:36px 32px">'
            f'<p style="font-size:{fs}px;{BF};color:{c};line-height:1.1;white-space:nowrap">{num}</p><p style="font-size:{ts}px;line-height:1.5">{t}</p></div>')
def table(rows, widths, fs=30, hl=None):
    h = ''.join(f'<th style="width:{w};text-align:left">{c}</th>' for c, w in zip(rows[0], widths))
    out = f'<table style="font-size:{fs}px;color:#404040"><tr>{h}</tr>'
    for i, r in enumerate(rows[1:], 1):
        st = ' style="background:#F2F2F2"' if hl == i else ''
        out += f'<tr{st}>' + ''.join(f'<td>{c}</td>' for c in r) + '</tr>'
    return out + '</table>'

slides = {}

slides['prev'] = sec('prev', 'Previous Work', '기존 연구방향 :: PINN 입도예측 + 워크플로우 검증',
    row(card(head('01&nbsp; PINN 기반 입도예측', 44), bul(f'입력: 시뮬레이션 {B("하중·변위")} 데이터', 36), bul(f'출력: 분쇄 후 {B("입도분포")}', 36), center=True),
        card(head('02&nbsp; 풀시퀀스 워크플로우 검증', 44), bul('로봇 파지 → 크러셔 삽입·분쇄 → 회수', 36), bul(f'{B("디지털트윈 재현")} ↔ 실기 대조', 36), center=True), gap=32, grow=True) +
    frame('기존 파이프라인', flow(gbox('Crusher 하드웨어', fs=34), gbox('시뮬레이션<br>하중·변위 데이터', fs=34), gbox('PINN 학습', fs=34), gbox('입도분포 예측', True, 34)), h='padding-top:60px;padding-bottom:44px;'),
    '원래 계획은 두 축이었습니다. 하나는 시뮬레이션 데이터로 PINN 을 학습해 입도를 예측하는 것, 다른 하나는 로봇 파지부터 회수까지 전체 워크플로우를 디지털트윈에서 검증하는 것이었습니다.')

slides['prob1'] = sec('prob1', 'Problem', '발생한 문제 ① :: 물리적 충실도 부족',
    head('보기 좋은 시뮬레이션 ≠ 물리적으로 옳은 시뮬레이션', 44) +
    bul('디지털트윈 영상: 그럴듯함', 34) + bul(f'힘 데이터: {B("물리적 충실도 매우 낮음")}', 34) + bul(f'PINN 학습 입력 오류 → {B("예측 무의미")}', 34) +
    row(stat('×22.4', 'IPC 입자 접촉력 / 운동량 수지 (중앙값)<br>자중 대비 수십 배 왜곡'),
        stat('92–96 %', '서로 한 번도 닿지 않은 입자 쌍<br>barrier 거리에 정지', red=False), gap=32, grow=True) +
    '<p style="font-size:24px;color:#7F7F7F">출처: Genesis IPC 500알, 60 s @ 8 rpm (Fig. 5c)</p>',
    '첫 번째 문제는 시뮬레이션이 보기에는 좋았지만 물리적으로 옳지 않았다는 점입니다. IPC 점질량 파우더에서 접촉력과 운동량 수지의 비가 중앙값 22배, 입자 쌍의 92~96%는 barrier 거리에 멈춰 한 번도 닿지 않았습니다. 이 데이터를 PINN 에 넣는 것은 의미가 없습니다.')

slides['prob2'] = sec('prob2', 'Feedback', '임세혁 박사님 피드백 :: Digital Twin 의 당위성',
    row(col(head('01&nbsp; 센싱 부재', 36), gray('현실 상태를 모르면 시뮬레이션 의미 약화'),
            bul(f'샘플백 내 {B("파우더 분포")} 미측정'), bul('초기 상태부터 현실과 오차'),
            '<div style="height:12px"></div>', tag('대응 방향'), gbox('초기 상태 센싱 (카메라)')), VR,
        col(head('02&nbsp; 공정 개입 부재', 36), gray('시뮬레이션 결과가 공정 방향을 결정해야 함'),
            bul('예: 카메라로 입도 확인'), bul(f'덜 부서졌으면 → {B("추가 분쇄")}'),
            '<div style="height:12px"></div>', tag('대응 방향'), gbox('입도 피드백 → 추가 분쇄 루프'))) +
    '<div style="height:24px"></div>' +
    key('공통 과제', f'센싱 + 공정 개입 → {B("Digital Twin 의 당위성")} 확보'),
    '두 번째는 임세혁 박사님 피드백입니다. 첫째, 샘플백 안 파우더 분포 같은 현실 상태를 센싱하지 않으면 시뮬레이션은 시작부터 현실과 어긋나 있고, 그러면 디지털트윈으로서 의미가 약합니다. 둘째, 디지털트윈이라면 시뮬레이션 결과가 공정에 개입해 방향을 바꿔야 합니다. 예를 들어 카메라로 입도를 보고 덜 부서졌으면 더 부수는 식입니다.')

slides['defense'] = sec('defense', 'Feedback', '그래서 내린 결정 :: 디펜스 가능한 형태로 마무리',
    f'<div style="display:flex;flex-direction:column;gap:20px;background:{RBG};border:1px solid {RBD};padding:56px 64px">'
    f'<p style="font-size:26px;color:{RED}">임세혁 박사님</p>'
    f'<p style="font-size:56px;line-height:1.4;{BF};color:#262626">“당장의 졸업엔 문제없지만,<br>이후 연구를 디펜스할 때 항상 걸림돌이 될 것”</p></div>' +
    '<div style="height:16px"></div>' +
    bul(f'일정 지연 감수', 32) + bul(f'{B("물리적 정합성")} + {B("Digital Twin 당위성")} 확보 후 마무리', 32),
    '박사님께서 당장의 졸업에는 큰 문제가 없지만 나중에 이 연구를 디펜스할 때 계속 걸림돌이 될 거라고 하셨습니다. 그래서 시간이 좀 더 걸리더라도 제대로 마무리하는 쪽으로 결정했습니다.')

slides['hw'] = sec('hw', 'Current Work', '하드웨어 점검 :: 풀시퀀스 구동 이상 없음',
    row(col(head('Hardware check', 30),
            table([['기구', '상태'], ['로봇팔 (M1013)', '정상'], ['석션 모듈', '정상'], ['크러셔', '정상'], ['로드셀 센싱', '정상'], ['홀더 / 회수장치', '정상']], ['62%', '38%']),
            '<div style="height:12px"></div>', key('결과', f'전 기구 {B("이상 없음")}')),
        col('<img alt="[실기 풀시퀀스 사진 또는 영상]" style="width:860px;height:600px;background:#F2F2F2">', cap('실기 풀시퀀스 구동'), w='width:860px'), align='center'),
    '먼저 풀시퀀스를 돌리기 위해 모든 기구가 제대로 작동하는지 확인했습니다. 로봇팔, 석션 모듈, 크러셔, 로드셀, 홀더 모두 이상 없었습니다. 오른쪽에는 실기 풀시퀀스 사진이나 영상을 넣으면 됩니다.')

slides['ipc'] = sec('ipc', 'Current Work', '물리적 정합성 :: IPC 커플러 제거',
    bul(f'IPC potential barrier: 관통 원천 차단 → {B("“보기 좋은” 솔버·커플러")}') +
    bul(f'표면 접촉 전 힘 발생 ({B("+81 µm 에서 0.22 N")}) · 실제 접촉 미형성') +
    frame('Fig. 5 &nbsp;Genesis IPC 점질량 vs DEM',
          img('/_blob/c11be2bdceb46341869e87ea8a7745ff', 'IPC vs DEM: (a) 최근접 표면 간격 분포, (b) 접촉력-간격 법칙', 1208, 400) +
          cap('(a) 최근접 간격: IPC 0.1d 에 정지, DEM 0 에 집중 &nbsp;·&nbsp; (b) 접촉력 법칙: IPC 는 떨어진 상태에서 힘 발생')) +
    key('결론', f'하중 기반 예측 불가 → {B("배선에서 제거")}'),
    '물리 정합성을 위해 처음 한 일은 IPC 커플러를 배선에서 빼는 것이었습니다. IPC 는 barrier 로 관통을 막아서 보기에는 좋지만, 입자가 닿기 전에 힘이 생기고 실제 접촉은 형성되지 않습니다. 왼쪽 그래프에서 IPC 는 간격 0.1d 에 몰려 있고 DEM 은 0 에 몰려 있습니다.')

slides['cost'] = sec('cost', 'Current Work', '계산비용 :: FEM 샘플백 결합의 한계',
    bul(f'IPC: dt 5 ms = 접촉시간 10 µs × {B("487")} → 접촉 미해상, {B("저비용은 착시")}') +
    bul(f'접촉 해상 dt + FEM 샘플백 동시 계산: 스트로크당 {B("14,932 s")} (사전 런 1,835 s 의 {R("8.1배")})') +
    frame('Fig. 6 &nbsp;결합 vs 사전 런 (RTX 5080, 5,824알)',
          img('/_blob/a119220b272835e4baeaebf3c90c4740', '결합 FEM 샘플백+DEM vs 사전 런+DEM: (a) 스트로크당 계산시간, (b) 누적 계산시간', 1243, 400) +
          cap('(a) 스트로크당 계산시간 &nbsp;·&nbsp; (b) 스트로크 수에 따른 누적 계산시간')) +
    key('결론', f'샘플백: 사전 런 1회 → {B("운동학 경계로 재생")} &nbsp;·&nbsp; 입자: {B("DEM")}'),
    '그래도 경향성은 볼 수 있지 않을까 해서 FEM 샘플백과 점질량 조합을 더 밀어봤지만 계산량이 문제였습니다. IPC 가 싸 보였던 건 dt 가 접촉시간의 487배라 접촉을 해상하지 않기 때문이고, 접촉을 해상하는 dt 로 샘플백까지 같이 풀면 한 스트로크에 4시간이 넘습니다. 그래서 샘플백은 한 번 풀어 재생하고, 입자는 DEM 으로 푸는 구조로 바꿨습니다.')

slides['dem'] = sec('dem', 'Current Work', 'DEM 채택 :: MPM 과 한 스트로크 비교',
    row(frame('한 스트로크 압착 · 3D 시각화',
              img('/_blob/fda41a7f784e2612d79c7b55516e094f', '한 스트로크 DEM vs MPM 3D 애니메이션: 입자 접촉력 색상, 샘플백은 녹화 재생 경계', 622, 350) +
              cap('좌: DEM (입자 접촉력 색상) &nbsp;·&nbsp; 우: MPM') +
              f'<p style="font-size:22px;color:{RED};text-align:center">※ 샘플백 = 사전 런 녹화 재생 (운동학 경계, 동역학 미계산)</p>', w='width:682px'),
        frame('압착축 단면 · 2D',
              img('/_blob/9cdeca776862c0e56bc37175c5873ffb', '한 스트로크 DEM vs MPM 압착축 단면 애니메이션: 입자 접촉력 색상', 787, 350) +
              cap('좌: DEM &nbsp;·&nbsp; 우: MPM'), w='flex:1'), gap=32) +
    '<div style="height:4px"></div>' +
    row(col(table([['한 스트로크 (3 g, 5,824알)', 'MPM', 'DEM'], ['계산시간', '642 s', '1,825 s (2.8배)'], ['간격 &lt; 2 mm', '판 통과', f'{B("판 stall")}'], ['입자별 하중', '없음 (연속체)', f'{B("있음")}']],
                  ['38%', '28%', '34%'], fs=26)),
        col(key('채택', f'입자별 접촉하중 → {B("파쇄 모델 입력")}'),
            '<p style="font-size:22px;color:#7F7F7F">8 rpm, RTX 5080 · Newton(Warp) 이관 후 DEM 직접 구현</p>', w='width:620px'), gap=48, align='center'),
    '그래서 채택한 것이 입자 표현에 범용적으로 쓰이는 DEM 입니다. Genesis 에는 DEM 이 배선되어 있지 않아 플랫폼을 옮기고 시퀀스와 배선을 전면 수정했습니다. 같은 조건에서 MPM 과 비교하면 계산시간은 같은 자릿수이고, MPM 은 간격이 2 mm 아래로 가면 파우더가 판을 통과하지만 DEM 은 입자층이 걸려 판이 멈춥니다. 무엇보다 입자별 접촉하중이 나와서 파쇄 모델의 입력이 됩니다.')

slides['lit1'] = sec('lit1', 'Novelty', '선행 연구 :: 반복 충격과 손상 누적',
    row(col(head('Tavares &amp; King (2002)<br>Tavares &amp; Carvalho (2009)', 30),
            bul('단일 vs 다회 stroke 파쇄 차이 규명 (CDM)'), bul(f'파쇄 비에너지 (J/kg) {B("∝ stiffness")}'), bul(f'stiffness {B("∝ 누적 손상")} → 충격마다 감소'),
            '<div style="height:12px"></div>', tag('직접 구현'),
            gbox('대리석 4.4 mm, 7번째 충격에서 파쇄<br><span style="font-family:\'Pretendard\', Arial, sans-serif">집단 누적파쇄율: 해석식과 0.3 %p 이내</span>', True, 26), w='width:640px'),
        frame('Newton DEM 구현 · 반복충격 CDM',
              img('/_blob/e3a5930830a09bd3236ab68e3b228488', 'Tavares and King 2002 반복충격 CDM 구현: 7번째 충격에서 대리석 입자 파쇄', 1000, 563, ';background:#14213D', '/_blob/75a77b12dbe2cc4b373becf5f20ce350') +
              cap('Ek = 3.1 mJ (논문 Fig. 2 캡션 1.4 mJ 는 오기로 판단)'), w='flex:1'), gap=48, align='center'),
    '두 번째 축은 노벨티입니다. 2002년, 2009년 Tavares 그룹 논문은 단일 스트로크와 여러 번 스트로크로 깨는 것의 차이를 규명했습니다. 핵심은 파쇄에 드는 비에너지가 stiffness 에 비례하고, 그 stiffness 가 손상이 쌓이면서 떨어진다는 것입니다. 직접 구현해 보니 7번째 충격에서 깨지고, 집단 누적파쇄율이 논문 식과 0.3%p 안에서 맞았습니다. 참고로 논문 Fig.2 캡션의 1.4 mJ 로는 40번에도 안 깨져서 3.1 mJ 를 썼습니다.')

slides['lit2'] = sec('lit2', 'Novelty', '선행 연구의 공백 :: 반복압밀의 입도분포',
    table([['연구', '밝힌 것', '다루지 않은 것'],
           ['Tavares 그룹 (2002, 2009)<br>반복충격 CDM', f'손상 누적에 따른 파쇄 {B("확률")}', f'파쇄 입자의 {B("분포")}'],
           ['Esnault 외 (2015)<br>압축 분쇄 (piston-die) PBM', '1회 압축 시 입도분포 이동 (bed porosity 기반)', f'{B("반복")} 압밀'],
           [R('본 연구'), '반복압밀 + 손상누적 + PBM', f'{B("로봇 시뮬레이션 개체군 입도분포")}']], ['30%', '35%', '35%'], hl=3) +
    '<div style="height:8px"></div>' +
    bul(f'가설: 단일 vs 반복 압밀 시 파쇄 정도 차이 ↔ Tavares 논리와 {B("일치")}') +
    key('공백', f'두 연구를 엮어도 {B("반복압밀의 입도분포")}는 비어 있음') +
    '<p style="font-size:22px;color:#7F7F7F">PBM = population balance model (개체군 수지)</p>',
    'Tavares 논리는 제 가설, 즉 단일 스트로크보다 반복 압밀에서 부서지는 정도가 다를 거라는 가설과 맞습니다. 다만 Tavares 는 파쇄 확률을 모델링할 뿐 파쇄된 입자들이 어떤 분포를 갖는지는 다루지 않습니다. 그 부분은 2015년 die compression + PBM 논문과 엮습니다. 이 논문은 개체군이 파쇄되며 어느 크기 구간으로 이동하는지를 다이 압밀에서 밝혔지만, 반복 압밀은 다루지 않았습니다.')

slides['novelty'] = sec('novelty', 'Novelty', '본 논문의 노벨티',
    f'<div style="border:2px solid {RED};background:{RBG};padding:56px 40px">'
    f'<p style="font-size:64px;line-height:1.45;{BF};color:#262626;text-align:center">로봇 시뮬레이션 기반<br>{R("반복압밀")} 개체군의 {R("입도분포")} 계산</p></div>' +
    '<div style="height:40px"></div>' +
    flow(gbox('DEM<br><span style="font-family:\'Pretendard\', Arial, sans-serif;font-size:22px">입자별 충격에너지</span>'),
         gbox('CDM<br><span style="font-family:\'Pretendard\', Arial, sans-serif;font-size:22px">손상누적 파쇄 판정</span>'),
         gbox('PBM<br><span style="font-family:\'Pretendard\', Arial, sans-serif;font-size:22px">스트로크별 입도분포 갱신</span>'),
         gbox('체질 PSD<br><span style="font-family:\'Pretendard\', Arial, sans-serif;font-size:22px">실물 검증</span>', True)),
    '따라서 이 논문의 노벨티는 로봇 시뮬레이션에서 반복압밀을 통한 개체군의 입도분포를 계산하는 과정입니다. DEM 이 입자별 충격에너지를 주고, CDM 손상누적으로 파쇄를 판정하고, PBM 으로 스트로크마다 입도분포를 갱신하고, 실물은 체질로 검증합니다.')

slides['sensor'] = sec('sensor', 'Experiment', '실험 센싱 :: 카메라 + 슬라이더 변위센서',
    row(card(head('01&nbsp; 카메라'), gray('피드백 ① 센싱 · ② 공정 개입'),
            bul(f'샘플백 내 파우더 {B("초기 분포")}'), bul(f'분쇄 후 {B("입도")}'),
            BOTTOM, tag('용도'), gbox('시뮬레이션 초기조건<br>공정 개입 판단')),
        card(head('02&nbsp; 슬라이더 변위센서'), gray('시뮬레이션 대조'),
            bul(f'스트로크별 {B("판 변위")}'), bul(f'로드셀 반력 + 변위 → {B("반력-위치 곡선")}'),
            BOTTOM, tag('용도'), gbox('시뮬레이션<br>반력-위치 곡선과 대조')),
        card(head('03&nbsp; 공정 개입 루프'), gray('Digital Twin 의 당위성'),
            BOTTOM, gbox('카메라 입도 확인'), ARD, gbox('시뮬레이션과 비교'), ARD, gbox('입도 미달 → 추가 분쇄', True)), gap=28, grow=True) +
    key('센서 구성', f'기존: 로드셀 &nbsp;·&nbsp; {B("추가: 카메라, 슬라이더 변위센서")}'),
    '실제 실험에 들어가야 할 센서는 카메라와 슬라이더 변위센서 하나입니다. 카메라는 박사님 첫 번째 피드백인 초기 분포 센싱과, 두 번째 피드백인 입도 확인 후 공정 개입에 모두 쓰입니다. 변위센서는 로드셀과 함께 반력-위치 곡선을 실측해 시뮬레이션과 대조합니다.')

slides['plan'] = sec('plan', 'Plan', '논문 투고 계획 :: 일정 수정',
    row(card(head('투고 계획 수정', 60), bul(f'Digital Twin 당위성 {B("논리 보완")}', 44), bul(f'보완 후 {B("수정 일정 재보고")}', 44), center=True), grow=True) +
    f'<div style="background:{RBG};border:1px solid {RBD};padding:44px 48px"><p style="font-size:48px;{BF};color:{RED};text-align:center">기존 계획대로 진행하지 못해 죄송합니다.</p></div>',
    '마지막으로 논문 투고 계획이 수정되었습니다. Digital Twin 의 당위성을 논리적으로 가다듬은 후에 수정된 일정과 함께 다시 말씀드리겠습니다. 계획대로 진행하지 못해 죄송하다는 말씀 드립니다.')

# ── 14장 → 11장: prob2+defense, ipc+cost, lit2+novelty 를 합친다 ──
for k in ('defense', 'cost', 'novelty'):
    slides.pop(k)

slides['prob2'] = sec('prob2', 'Feedback', '임세혁 박사님 피드백 :: Digital Twin 의 당위성',
    row(card(head('01&nbsp; 센싱 부재'),
            bul(f'샘플백 내 {B("파우더 분포")} 미측정'), bul('초기 상태부터 현실과 오차'),
            BOTTOM, tag('대응 방향'), gbox('초기 상태 센싱 (카메라)')),
        card(head('02&nbsp; 공정 개입 부재'),
            bul('시뮬레이션 결과 → 공정 방향 결정 필요'), bul(f'예: 입도 확인 → 미달 시 {B("추가 분쇄")}'),
            BOTTOM, tag('대응 방향'), gbox('입도 피드백 → 추가 분쇄 루프')), gap=32, grow=True) +
    frame('임세혁 박사님',
          f'<p style="font-size:40px;line-height:1.4;{BF};color:#262626;text-align:center">“당장의 졸업엔 문제없지만, 이후 연구를 디펜스할 때 항상 걸림돌이 될 것”</p>'
          f'<p style="font-size:32px;text-align:center">→ 일정 지연 감수, {B("물리적 정합성")} + {B("Digital Twin 당위성")} 확보 후 마무리</p>', h='padding:44px 40px 26px;'),
    '두 번째는 임세혁 박사님 피드백입니다. 첫째, 샘플백 안 파우더 분포 같은 현실 상태를 센싱하지 않으면 시뮬레이션은 시작부터 현실과 어긋나 있습니다. 둘째, 디지털트윈이라면 시뮬레이션 결과가 공정에 개입해 방향을 바꿔야 합니다. 박사님께서 당장의 졸업에는 문제없지만 나중에 디펜스할 때 계속 걸림돌이 될 거라고 하셔서, 시간이 더 걸리더라도 제대로 마무리하기로 했습니다.')

slides['ipc'] = sec('ipc', 'Current Work', 'IPC 커플러 제거 :: 물리적 정합성 + 계산비용',
    row(col(head('01&nbsp; 물리적 정합성', 32),
            bul(f'barrier 가 관통 차단 → {B("“보기 좋은” 솔버")}', 26),
            bul(f'표면 접촉 전 힘 발생 ({B("+81 µm, 0.22 N")})', 26),
            frame('Fig. 5 &nbsp;IPC 점질량 vs DEM',
                  img('/_blob/c11be2bdceb46341869e87ea8a7745ff', 'IPC vs DEM: (a) 최근접 표면 간격 분포, (b) 접촉력-간격 법칙', 780, 258) +
                  cap('(a) 최근접 간격 &nbsp;·&nbsp; (b) 접촉력 법칙'))), VR,
        col(head('02&nbsp; 계산비용', 32),
            bul(f'IPC dt = 접촉시간 × {B("487")} → 저비용은 착시', 26),
            bul(f'FEM 샘플백 동시 계산: {B("14,932 s")}/스트로크 ({R("8.1배")})', 26),
            frame('Fig. 6 &nbsp;결합 vs 사전 런',
                  img('/_blob/a119220b272835e4baeaebf3c90c4740', '결합 FEM 샘플백+DEM vs 사전 런+DEM: (a) 스트로크당 계산시간, (b) 누적 계산시간', 780, 252) +
                  cap('(a) 스트로크당 계산시간 &nbsp;·&nbsp; (b) 누적 계산시간'))), gap=40) +
    key('결론', f'IPC 제거 &nbsp;→&nbsp; 샘플백: 사전 런 1회 {B("운동학 경계로 재생")} &nbsp;·&nbsp; 입자: {B("DEM")}'),
    '물리 정합성을 위해 IPC 커플러를 배선에서 뺐습니다. IPC 는 barrier 로 관통을 막아 보기에는 좋지만, 입자가 닿기 전에 힘이 생기고 실제 접촉은 형성되지 않습니다. 계산비용도 문제였습니다. IPC 가 싸 보였던 건 dt 가 접촉시간의 487배라 접촉을 해상하지 않기 때문이고, 접촉을 해상하는 dt 로 FEM 샘플백까지 같이 풀면 한 스트로크에 4시간이 넘습니다. 그래서 샘플백은 한 번 풀어 재생하고 입자는 DEM 으로 풉니다.')

slides['lit2'] = sec('lit2', 'Novelty', '본 논문의 노벨티 :: 반복압밀 개체군 입도분포',
    table([['연구', '밝힌 것', '다루지 않은 것'],
           ['Tavares 그룹 (2002, 2009) · 반복충격 CDM', f'손상 누적에 따른 파쇄 {B("확률")}', f'파쇄 입자의 {B("분포")}'],
           ['[저자] (2015) · Die compression + PBM', '다이 압밀 시 입도분포 이동', f'{B("반복")} 압밀']], ['38%', '34%', '28%'], fs=30) +
    f'<div style="flex:1;display:flex;align-items:center;justify-content:center;border:2px solid {RED};background:{RBG};padding:36px 40px">'
    f'<p style="font-size:60px;line-height:1.4;{BF};color:#262626;text-align:center">로봇 시뮬레이션 기반<br>{R("반복압밀")} 개체군의 {R("입도분포")} 계산</p></div>' +
    flow(gbox('DEM<br><span style="font-family:\'Pretendard\', Arial, sans-serif;font-size:26px">입자별 충격에너지</span>', fs=34),
         gbox('CDM<br><span style="font-family:\'Pretendard\', Arial, sans-serif;font-size:26px">손상누적 파쇄 판정</span>', fs=34),
         gbox('PBM<br><span style="font-family:\'Pretendard\', Arial, sans-serif;font-size:26px">스트로크별 입도분포 갱신</span>', fs=34),
         gbox('체질 PSD<br><span style="font-family:\'Pretendard\', Arial, sans-serif;font-size:26px">실물 검증</span>', True, 34)) +
    '<p style="font-size:24px;color:#7F7F7F">PBM = population balance model (개체군 수지)</p>',
    'Tavares 는 파쇄 확률을 모델링할 뿐 파쇄된 입자들의 분포는 다루지 않고, 2015년 die compression + PBM 논문은 분포 이동을 밝혔지만 반복 압밀은 다루지 않았습니다. 그래서 이 논문의 노벨티는 로봇 시뮬레이션에서 반복압밀을 통한 개체군의 입도분포를 계산하는 것입니다. DEM 이 입자별 충격에너지를 주고, CDM 으로 파쇄를 판정하고, PBM 으로 스트로크마다 분포를 갱신하고, 실물은 체질로 검증합니다.')

# ── 11장 → 7장: 말하려는 바가 겹치는 슬라이드끼리 한 장에 모은다 (2026-11-02 지도) ──
#   prev+prob1 → prev,  ipc+dem → ipc,  lit1+lit2 → lit2,  hw+sensor → sensor
for k in ('prob1', 'dem', 'lit1', 'hw'):
    slides.pop(k)
def sp(t, fs): return f'<span style="font-family:\'Pretendard\', Arial, sans-serif;font-size:{fs}px">{t}</span>'

slides['prev'] = sec('prev', 'Previous Work · Problem', '기존 연구방향 :: 시뮬레이션 힘 데이터의 충실도 문제',
    row(frame('기존 계획',
              col(head('01&nbsp; PINN 기반 입도예측', 32), bul(f'입력: 시뮬레이션 {B("하중·변위")} 데이터', 28), bul(f'출력: 분쇄 후 {B("입도분포")}', 28),
                  head('02&nbsp; 풀시퀀스 워크플로우 검증', 32), bul('로봇 파지 → 크러셔 삽입·분쇄 → 회수', 28), bul(f'{B("디지털트윈 재현")} ↔ 실기 대조', 28),
                  '<div style="height:8px"></div>',
                  flow(gbox('시뮬레이션<br>힘 데이터', fs=22), gbox('PINN<br>학습', fs=22), gbox('입도분포<br>예측', fs=22)), w='width:100%'),
              w='width:700px', h='align-items:stretch;padding:44px 36px 32px;'),
        frame('발생한 문제 ① · 보기 좋은 시뮬레이션 ≠ 물리적으로 옳은 시뮬레이션',
              col(bul('디지털트윈 영상: 그럴듯함', 28), bul(f'힘 데이터: {B("물리적 충실도 매우 낮음")} → PINN 입력 오류', 28),
                  img('/_blob/0358a6a30d03b1f0c3a6040c6e45e164', 'Genesis IPC 런: (a) 최근접 입자 표면 간격 분포, barrier 에 84% 정지, (b) 접촉력 대 운동량 수지 힘 비율, 중앙값 22.4배', 880, 370),
                  cap(f'(a) 입자 {B("84 %")}가 barrier 거리에 정지 &nbsp;·&nbsp; (b) 접촉력 / 운동량 수지 중앙값 {B("×22.4")}'),
                  '<p style="font-size:20px;color:#7F7F7F">Genesis IPC 500알 (d = 1 mm), 8 rpm × 60 s, 2,400 프레임</p>', w='width:100%;flex:1'),
              w='flex:1', h='align-items:stretch;padding:44px 40px 28px;'), gap=32, grow=True) +
    key('결론', f'PINN 의 입력인 시뮬레이션 힘 데이터를 믿을 수 없음 → {B("물리적 정합성 확보가 선행 과제")}'),
    '원래 계획은 시뮬레이션 하중·변위 데이터로 PINN 을 학습해 입도를 예측하고, 로봇 파지부터 회수까지 워크플로우를 디지털트윈에서 검증하는 것이었습니다. 그런데 시뮬레이션이 보기에는 좋았지만 물리적으로 옳지 않았습니다. 왼쪽 그래프는 입자와 가장 가까운 이웃 사이 간격인데, 실제 분말이면 0 에 몰려야 할 것이 barrier 거리 0.1 에 몰려 있습니다. 입자의 84% 가 서로 닿지 않고 떠 있다는 뜻입니다. 오른쪽은 솔버가 내놓는 접촉력을 입자의 실제 움직임에서 역산한 힘으로 나눈 비율인데, 1 이어야 할 값이 중앙값 22배입니다. PINN 의 입력 자체를 믿을 수 없으니 물리적 정합성 확보가 먼저입니다.')

slides['ipc'] = sec('ipc', 'Current Work', '물리적 정합성 :: IPC 제거 → DEM 채택',
    row(frame('Fig. 5 &nbsp;IPC 점질량 vs DEM · 정합성',
              img('/_blob/c11be2bdceb46341869e87ea8a7745ff', 'IPC vs DEM: (a) 최근접 표면 간격 분포, (b) 접촉력-간격 법칙', 725, 240) +
              cap(f'IPC: 표면 접촉 전 힘 발생 ({B("+81 µm, 0.22 N")}) · 입자 쌍 92–96 % 비접촉'), w='flex:1'),
        frame('Fig. 6 &nbsp;FEM 샘플백 결합 vs 사전 런 · 계산비용',
              img('/_blob/a119220b272835e4baeaebf3c90c4740', '결합 FEM 샘플백+DEM vs 사전 런+DEM: (a) 스트로크당 계산시간, (b) 누적 계산시간', 746, 240) +
              cap(f'IPC 저비용 = dt 가 접촉시간 ×487 (미해상) · 결합 시 {B("14,932 s")}/스트로크 ({R("8.1배")})'), w='flex:1'), gap=28) +
    row(frame('DEM vs MPM · 3D 시각화',
              img('/_blob/fda41a7f784e2612d79c7b55516e094f', '한 스트로크 DEM vs MPM 3D 애니메이션: 입자 접촉력 색상, 샘플백은 녹화 재생 경계', 446, 251) +
              f'<p style="font-size:20px;color:#595959;text-align:center">※ 샘플백 = 사전 런 녹화 재생 (동역학 미계산)</p>', w='width:520px'),
        frame('DEM vs MPM · 압착축 단면',
              img('/_blob/9cdeca776862c0e56bc37175c5873ffb', '한 스트로크 DEM vs MPM 압착축 단면 애니메이션: 입자 접촉력 색상', 565, 251) +
              cap('좌: DEM &nbsp;·&nbsp; 우: MPM'), w='width:630px'),
        col(table([['한 스트로크', 'MPM', 'DEM'], ['계산시간', '642 s', '1,825 s'], ['간격 &lt; 2 mm', '판 통과', B('판 stall')], ['입자별 하중', '없음', B('있음')]],
                  ['40%', '28%', '32%'], fs=24),
            key('채택', f'입자별 하중 → {B("파쇄 모델 입력")}', fs=24),
            '<p style="font-size:20px;color:#7F7F7F">3 g, 5,824알 · 8 rpm · RTX 5080</p>'), gap=28, align='center'),
    '물리 정합성을 위해 IPC 커플러를 뺐습니다. Fig. 5 처럼 IPC 는 barrier 로 관통을 막아 보기에는 좋지만, 입자가 닿기 전에 힘이 생기고 실제 접촉은 형성되지 않습니다. Fig. 6 처럼 IPC 가 싸 보였던 건 dt 가 접촉시간의 487배라 접촉을 해상하지 않기 때문이고, 접촉을 해상하는 dt 로 FEM 샘플백까지 같이 풀면 한 스트로크에 4시간이 넘습니다. 그래서 샘플백은 한 번 풀어 재생하고 입자는 DEM 으로 풉니다. MPM 과 비교하면 계산시간은 같은 자릿수이고, MPM 은 간격이 2 mm 아래에서 파우더가 판을 통과하지만 DEM 은 판이 멈추고, 무엇보다 입자별 하중이 나와 파쇄 모델의 입력이 됩니다.')

slides['lit2'] = sec('lit2', 'Novelty', '본 논문의 노벨티 :: 반복압밀 개체군 입도분포',
    row(col(table([['선행 연구', '밝힌 것', '다루지 않은 것'],
                   ['Tavares 그룹 (2002, 2009)<br>반복충격 CDM', f'비에너지 {B("∝ stiffness")}, stiffness {B("∝ 누적 손상")}<br>→ 파쇄 {B("확률")}', f'파쇄 입자의 {B("분포")}'],
                   ['Esnault 외 (2015)<br>압축 분쇄 (piston-die) PBM', '1회 압축 시 입도분포 이동 (bed porosity 기반)', f'{B("반복")} 압밀'],
                   [R('본 연구'), '반복압밀 + 손상누적 + PBM', B('로봇 시뮬레이션 개체군 입도분포')]], ['32%', '42%', '26%'], fs=26, hl=3)),
        frame('Tavares CDM 직접 구현 (Newton DEM)',
              img('/_blob/e3a5930830a09bd3236ab68e3b228488', 'Tavares and King 2002 반복충격 CDM 구현: 7번째 충격에서 대리석 입자 파쇄', 560, 315, ';background:#14213D', '/_blob/75a77b12dbe2cc4b373becf5f20ce350') +
              cap(f'대리석 4.4 mm {B("7번째 충격 파쇄")} · 해석식과 {B("0.3 %p")} 이내'), w='width:640px'), gap=36, align='center') +
    frame('본 논문의 노벨티',
          f'<p style="font-size:40px;line-height:1.3;{BF};color:#262626;text-align:center">로봇 시뮬레이션 기반 반복압밀 개체군의 입도분포 계산</p>', h='padding:34px 40px 18px;') +
    flow(gbox(f'DEM<br>{sp("입자별 충격에너지", 22)}', fs=28), gbox(f'CDM<br>{sp("손상누적 파쇄 판정", 22)}', fs=28),
         gbox(f'PBM<br>{sp("스트로크별 입도분포 갱신", 22)}', fs=28), gbox(f'체질 PSD<br>{sp("실물 검증", 22)}', True, 28)) +
    '<p style="font-size:20px;color:#7F7F7F">CDM 구현 Ek = 3.1 mJ (논문 Fig. 2 캡션 1.4 mJ 는 오기로 판단) · PBM = population balance model</p>',
    'Tavares 그룹은 단일 스트로크와 반복 스트로크 파쇄의 차이를 규명했습니다. 파쇄 비에너지가 stiffness 에 비례하고 stiffness 가 손상 누적으로 떨어진다는 것이고, 직접 구현해 보니 7번째 충격에서 깨지고 집단 누적파쇄율이 해석식과 0.3%p 안에서 맞았습니다. 다만 Tavares 는 파쇄 확률만 다루고 분포는 다루지 않으며, 2015년 die compression + PBM 논문은 분포 이동을 밝혔지만 반복 압밀은 다루지 않았습니다. 그래서 노벨티는 로봇 시뮬레이션에서 반복압밀을 통한 개체군의 입도분포를 계산하는 것입니다.')

slides['sensor'] = sec('sensor', 'Experiment', '실험 하드웨어 :: 점검 완료 + 센서 추가',
    row(frame('하드웨어 점검',
              col(table([['기구', '상태'], ['로봇팔 (M1013)', '정상'], ['석션 모듈', '정상'], ['크러셔', '정상'], ['로드셀', '정상'], ['홀더 / 회수장치', '정상']], ['62%', '38%'], fs=24),
                  key('결과', f'전 기구 {B("이상 없음")}', fs=24), w='width:100%'), w='width:470px', h='align-items:stretch;'),
        frame('실기 풀시퀀스',
              '<img alt="[실기 풀시퀀스 사진 또는 영상]" style="width:440px;height:400px;background:#F2F2F2">', w='width:500px'),
        frame('추가 센서 · 피드백 대응',
              col(head('01&nbsp; 카메라', 30), bul(f'샘플백 내 파우더 {B("초기 분포")} · 분쇄 후 {B("입도")}', 26),
                  tag('시뮬레이션 초기조건 · 공정 개입 판단'),
                  '<div style="height:1px;background:#D9D9D9;margin:6px 0"></div>',
                  head('02&nbsp; 슬라이더 변위센서', 30), bul(f'스트로크별 {B("판 변위")} + 로드셀 → {B("반력-위치 곡선")}', 26),
                  tag('시뮬레이션 반력-위치 곡선과 대조'), w='width:100%'), w='flex:1', h='align-items:stretch;padding:44px 36px 28px;'), gap=28, grow=True) +
    frame('공정 개입 루프 · Digital Twin 의 당위성',
          flow(gbox('카메라 입도 확인', fs=28), gbox('시뮬레이션과 비교', fs=28), gbox('입도 미달 → 추가 분쇄', True, 28)), h='padding:44px 28px 26px;') +
    key('센서 구성', f'기존: 로드셀 &nbsp;·&nbsp; {B("추가: 카메라, 슬라이더 변위센서")}', fs=28),
    '하드웨어는 풀시퀀스로 돌려 로봇팔, 석션, 크러셔, 로드셀, 홀더 모두 이상 없음을 확인했습니다. 실험에 추가할 센서는 카메라와 슬라이더 변위센서입니다. 카메라는 박사님 첫 번째 피드백인 초기 분포 센싱과 두 번째 피드백인 입도 확인 후 공정 개입에 쓰이고, 변위센서는 로드셀과 함께 반력-위치 곡선을 실측해 시뮬레이션과 대조합니다. 입도가 미달이면 추가 분쇄하는 루프가 디지털트윈의 당위성입니다.')

slides['plan'] = sec('plan', 'Plan', '논문 투고 계획 :: 일정 수정',
    row(card(head('투고 계획 수정', 56), bul(f'Digital Twin 당위성 {B("논리 보완")}', 40), bul(f'보완 후 {B("수정 일정 재보고")}', 40), center=True), grow=True) +
    f'<div style="background:{RBG};border:1px solid {RBD};padding:40px 48px"><p style="font-size:46px;{BF};color:{RED};text-align:center">기존 계획대로 진행하지 못해 죄송합니다.</p></div>',
    '마지막으로 논문 투고 계획이 수정되었습니다. Digital Twin 의 당위성을 논리적으로 가다듬은 후에 수정된 일정과 함께 다시 말씀드리겠습니다. 계획대로 진행하지 못해 죄송하다는 말씀 드립니다.')

# ── Tavares CDM 식 + DEM 개입 위치 (2026-11-02 요청: 식만, 설명은 발표자가) ──
# 식은 원문 PDF 대조: Tavares & King, Powder Technol. 123 (2002) 138–146 / Tavares, Powder Technol. 190 (2009) 327–339
def col2(*xs, w='flex:1'): return f'<div style="{w};display:flex;flex-direction:column;gap:8px">' + ''.join(xs) + '</div>'
MF = "font-family:'Times New Roman', Georgia, serif"
def eq(num, body, note=''):
    n = f'<p style="font-size:20px;color:#595959;width:118px;flex:none">{num}</p>'
    b = f'<p style="font-size:29px;{MF};color:#262626;flex:1">{body}</p>'
    t = f'<p style="font-size:20px;color:#7F7F7F;width:210px;flex:none">{note}</p>'
    return f'<div style="display:flex;align-items:center;gap:10px">{n}{b}{t}</div>'
def v(s): return f'<i>{s}</i>'
def frac(a, b): return (f'<span style="display:inline-flex;flex-direction:column;align-items:center;vertical-align:middle;margin:0 4px">'
                        f'<span style="border-bottom:1.5px solid #262626;padding:0 6px">{a}</span><span style="padding:0 6px">{b}</span></span>')
F_, d_, k_, a_, D_, g_, E_, m_, n_ = (v(s) for s in ('F', 'd', 'k', 'α', 'D', 'γ', 'E', 'm', 'n'))
kt = k_ + '&#771;'
km1, kn = f'{k_}<sub>{n_}−1</sub>', f'{k_}<sub>{n_}</sub>'
Em1, En = f'{E_}<sub>{n_}−1</sub>', f'{E_}<sub>{n_}</sub>'
Ds, ac, Ekn = f'{D_}*<sub>{n_}</sub>', f'{a_}<sub>c</sub>', f'{E_}<sub>k,{n_}</sub>'
sqd = f'{d_}<sup>1/2</sup>'
NUMH = '<div style="display:flex;gap:10px"><p style="font-size:18px;color:#7F7F7F;white-space:nowrap">식 번호 2002 · 2009</p></div>'
slides['cdm'] = sec('cdm', 'Novelty', '선행 연구 :: Tavares 반복충격 손상 모델 (CDM)',
    row(frame('Tavares &amp; King (2002) · Tavares (2009)',
              col2(NUMH,
                  '<p style="font-size:21px;color:#595959">단일 충격 · 손상</p>',
                  eq('(1) · (1)', f'{F_} = {frac(sqd, "3")} {kt} {a_}<sup>3/2</sup>', '접촉력 – 압축량'),
                  eq('(2) · (2)', f'{kt} = {k_} (1 − {D_})', '손상된 강성'),
                  eq('(3) · (3)', f'{D_} = ({a_} / {ac})<sup>{g_}</sup>', '비가역, 최댓값 D*'),
                  eq('(4) · (4)', f'{kn} = {km1} (1 − {Ds})', '충격 후 남는 강성'),
                  '<div style="height:1px;background:#D9D9D9;margin:4px 0"></div>',
                  '<p style="font-size:21px;color:#595959">반복 충격 · 누적</p>',
                  eq('(9) · (A.5)', f'{Em1} = {frac(f"4{g_} {sqd} {km1} {ac}<sup>5/2</sup>", f"15(2{g_}+5) {m_}<sub>p</sub>")}', '파쇄 비에너지 ∝ 강성'),
                  eq('(10) · (5)', f'{En} = {Em1} (1 − {Ds})', '충격마다 감소'),
                  eq('(11) · (6)', f'{Ds} = [ {frac(f"2{g_}", f"2{g_} − 5{Ds} + 5")} {frac(Ekn, Em1)} ]<sup>2{g_}/5</sup>', '충격 n 의 손상'),
                  eq('파쇄', f'{Ekn} &gt; {Em1}', ''),
                  eq('— · (13)', f'{v("t")}<sub>10</sub> = {v("A")} [1 − exp(−{v("b")}′ {v("E")}<sub>k</sub> / {v("E")}<sub>50b</sub>)]', '파편 입도'),
                  f'<p style="font-size:19px;color:#7F7F7F;margin-top:2px">{d_} 입자 지름 · {k_} 강성 · {a_} 압축량 · {ac} 파쇄 압축량 · {g_} 손상 축적 계수 · {E_}<sub>k,n</sub> n번째 타격 비에너지 · {m_}<sub>p</sub> 입자 질량</p>',
                  w='width:100%'), w='width:1010px', h='align-items:stretch;padding:38px 32px 16px;gap:6px;'),
        frame('DEM 1 스텝에서의 개입 위치',
              col2(gbox(f'접촉 탐색 → 겹침 {v("δ")}', fs=22), ARD,
                  gbox(f'접촉력: 선형 → 손상 강성 Hertz<br>{sp("식 (1)(2)", 20)}', True, 22), ARD,
                  gbox(f'입자별 손상 갱신<br>{sp("식 (3) · α = 가장 깊은 두 접촉 겹침 합", 20)}', True, 22), ARD,
                  gbox(f'접촉 종료 시 강성 저장<br>{sp("식 (4) · 다음 스트로크로 이월", 20)}', True, 22), ARD,
                  gbox(f'α ≥ α<sub>c</sub> → 파쇄 → 파편 교체<br>{sp("t<sub>10</sub> (2009 식 13) → PBM", 20)}', True, 22),
                  '<p style="font-size:19px;color:#7F7F7F;text-align:center">굵은 테두리 = DEM 에 추가 · 2002 식 (9)–(11) = 검증 해석해</p>',
                  w='width:100%'), w='flex:1', h='align-items:stretch;padding:38px 24px 16px;'), gap=28, grow=True),
    '제가 공부한 Tavares 그룹의 반복충격 손상 모델입니다. 식 번호는 2002 논문과 2009 논문을 같이 적었습니다. 식 (1)부터 (4)는 한 번의 충격에서 손상이 쌓이고 강성이 떨어지는 과정이고, 2002 식 (9)부터 (11), 2009 식 (5)와 (6)은 그 결과 파쇄 비에너지가 충격마다 줄어드는 과정입니다. 2009 식 (13)은 깨질 때 파편 입도입니다. 오른쪽은 이 식들이 DEM 한 스텝의 어디에 들어가는지입니다.')

# ── Esnault 2015 §5.4–5.5 (Rittinger + 투입 에너지) → 크랭크 F(θ) 로 옮긴 것 (2026-11-03 요청) ──
# 원문: Esnault, Zhou, Heitzmann, Minerals Eng. 73 (2015) 7–15. 식 (6) 피적분의 추가 x 는 단위상 오기 (해설 노트 동일).
# DEM 수치: crusher_scene/crush_out/dem.npz ctrl 열 — 판 변위 적분과 θ 적분이 0.11840 J 로 일치, 질량 3.00 g.
P_, x_, h_, rho, ph, th_, r_, L_, A_, W_, SS = (v(s) for s in ('P', 'x', 'h', 'ρ', 'φ', 'θ', 'r', 'L', 'A', 'W', 'SS'))
INT = '<span style="font-size:1.25em">∫</span>'
dSSdW, dWdph = frac(f'∂{SS}', f'∂{W_}'), frac(f'∂{W_}', f'∂{ph}')
sqk = f'√({L_}<sup>2</sup> − {r_}<sup>2</sup>sin<sup>2</sup>{th_})'
def eqw(num, body, note=''):  # cdm 의 eq 보다 번호 칸이 좁고 설명 칸이 넓다
    n = f'<p style="font-size:20px;color:#595959;width:70px;flex:none">{num}</p>'
    b = f'<p style="font-size:27px;{MF};color:#262626;flex:1">{body}</p>'
    t = f'<p style="font-size:19px;color:#7F7F7F;width:230px;flex:none">{note}</p>'
    return f'<div style="display:flex;align-items:center;gap:8px;width:100%">{n}{b}{t}</div>'
slides['esn'] = sec('esn', 'Novelty', '선행 연구 :: Esnault (2015) 압축분쇄 PBM → 크랭크 F(θ) 적용',
    row(frame('Esnault 외 (2015) · 강체 다이 (piston-die)',
              col2(eqw('(5)', f'{SS}({v("F")}<sub>n+1</sub>) − {SS}({v("F")}<sub>n</sub>) = {dSSdW} {dWdph} ({ph}<sub>n+1</sub> − {ph}<sub>n</sub>)', '새 표면적 = 한 일 (Rittinger)'),
                   eqw('(6)', f'{W_} = {frac("1", m_)} {INT}<sub>0</sub><sup>{x_}<sub>fin</sub></sup> {A_} {P_}({x_}) d{x_}', '투입 에너지 [J/kg]'),
                   eqw('(S5)', f'{dWdph} = − {frac(P_, f"{rho} (1 − {ph})<sup>2</sup>")}', f'{m_} = {rho}{A_}{h_}(1 − {ph}) → {B("강체 다이 전제")}'),
                   '<p style="font-size:20px;color:#7F7F7F;margin-top:4px">시간 변수 = 공극률 φ · 고정 B / 가변 B(φ) · 식 (6) 피적분의 추가 x 는 단위상 오기</p>',
                   w='width:100%'), w='flex:1', h='align-items:stretch;padding:38px 26px 14px;'),
        frame('본 시스템 · 크랭크-슬라이더 + 샘플백',
              col2(eqw('x(θ)', f'{x_} = {r_}(1 − cos{th_}) + {L_} − {sqk}', 'r 20 · L 80 mm'),
                   eqw('W', f'{W_}<sub>k</sub> = {frac("1", m_)} {INT}<sub>stroke k</sub> {v("F")}({th_}) {frac(f"d{x_}", f"d{th_}")} d{th_}', f'스트로크마다 {B("θ 로 적분")}'),
                   eqw('', f'&nbsp;&nbsp;&nbsp;&nbsp;= {frac("1", m_)} {INT} τ<sub>load</sub>({th_}) d{th_}', '모터 토크로 교차검증'),
                   f'<p style="font-size:22px;color:#404040;margin-top:4px">DEM 1 스트로크: 0.118 J / 3 g = {B("39.5 J/kg")} · Esnault ~10<sup>3</sup> J/kg → {B("반복 필수")}</p>',
                   w='width:100%'), w='flex:1', h='align-items:stretch;padding:38px 26px 14px;'), gap=28) +
    frame('공극률 φ 를 쓸 수 없을 때 — 샘플백 (측면 구속 없음)',
          row(col(img('/_blob/e26fe63f8218394d461ade6b917a3c5c', 'Newton DEM 렌더: 누르면 판 아래 알이 판 밖과 위로 밀려나 단층이 된다', 940, 334) +
                  cap(f'Newton DEM · 판 아래 알 {B("27 %")} 가 옆으로 밀려남'), w='width:940px'),
              table([['Esnault', '본 시스템'],
                     ['시간 변수 = φ', B('누적 W = Σ W<sub>k</sub>')],
                     ['∂W/∂φ 로 환산', f'불필요 · ΔSS = (∂SS/∂W) {B("ΔW")}'],
                     ['층 환경 = φ 하나', B('DEM 입자별 하중 → CDM')],
                     ['W 전부 → 표면', f'W − {B("W<sub>빈 백</sub>")} − W<sub>측면 유동</sub>']],
                    ['40%', '60%'], fs=21, hl=1), gap=28, align='center'),
          h='align-items:stretch;padding:34px 22px 8px;'),
    'Esnault 2015 의 5.4–5.5 절입니다. 새로 생긴 표면적을 한 일과 같다고 두고, 그 일을 피스톤 압력과 변위로 적분합니다. 강체 다이라서 층 높이가 곧 공극률이고, 그래서 φ 를 시간 변수로 씁니다. 저희 시스템은 판이 크랭크로 움직이므로 같은 일을 F(θ)와 dx/dθ 로 θ 에 대해 적분했고, DEM 한 스트로크는 39.5 J/kg 입니다. 샘플백은 측면 구속이 없어 φ 를 쓸 수 없으므로, 시간 변수를 누적 에너지로 바꾸고 층 환경은 DEM 입자별 하중으로 대신합니다.')

# ── 마지막 장: 연구 정리 1–4 + PINN 의 자리 (2026-11-03 요청) ──
def stepcol(title, *items):
    return frame(title, col(*[bul(t, 23) for t in items], w='width:100%'), w='flex:1', h='align-items:stretch;padding:36px 18px 14px;')
slides['summary'] = sec('summary', 'Summary', '정리 :: 반복압밀 입도분포 예측 — DEM · CDM · PBM · PINN',
    row(stepcol('① 문제 → 해결',
                f'IPC 접촉력 ≠ 운동량 수지 ({B("×22")})',
                f'IPC 제거 → {B("DEM 채택")}',
                '정합성 + 계산비용 확보 (완료)'),
        stepcol('② 선행연구 공백',
                f'Tavares: 손상누적 {B("확률")}만',
                f'Esnault: {B("1회")} 압축 · 강체 다이 φ',
                f'샘플백: φ 불가 ({B("27 %")} 측면 유출)'),
        stepcol('③ 결합 모델',
                f'시간 변수 φ → {B("누적 W")} (F(θ) 적분)',
                f'층 환경 φ → {B("DEM 하중")} → CDM',
                f'파편 t<sub>10</sub> → {B("PBM")} 스트로크별 갱신'),
        stepcol('④ 실기 검증',
                f'ForceGage + 모터 토크 → {B("W")}',
                f'빈 백 스트로크 → {B("백 에너지 제거")}',
                f'스트로크별 {B("체질 PSD")} 비교'), gap=20) +
    frame('PINN 의 자리 — DEM–CDM 대리모델 + S · B 역추정',
          flow(gbox(f'DEM–CDM<br>{sp("수 스트로크", 21)}', fs=26),
               gbox(f'PINN<br>{sp("물리 손실: 질량보존 · Rittinger · CDM 비가역", 21)}', True, 26),
               gbox(f'100 스트로크<br>{sp("PSD 외삽", 21)}', fs=26),
               gbox(f'실기 체질<br>{sp("역보정 S(W, x) · B · η", 21)}', fs=26)) +
          f'<p style="font-size:21px;color:#7F7F7F">DEM 1,825 s/스트로크 × 100 ≈ {B("51 h/조건", "#595959")} · 체질 데이터는 드문드문 → 데이터 손실 + 물리 손실 · 순방향 PBM 은 행렬곱이라 PINN 불필요</p>',
          h='padding:38px 30px 16px;') +
    key('본 연구', f'로봇 크러셔 반복압밀 · {B("누적 에너지 W 기반 PBM + 입자 CDM + PINN")} → 스트로크별 입도분포 예측', fs=30),
    '정리하겠습니다. 첫째, IPC 결합 시뮬레이션의 힘 데이터는 운동량 수지와 맞지 않아 DEM 으로 바꿨습니다. 둘째, Tavares 는 손상누적 파쇄 확률만, Esnault 는 1회 압축의 입도분포만 다루고, 샘플백에서는 공극률을 쓸 수 없습니다. 셋째, 그래서 시간 변수를 크랭크 F(θ)로 구한 누적 에너지로, 층 환경을 DEM 입자별 하중과 CDM 으로 바꿔 PBM 과 결합합니다. 넷째, 실기에서 힘과 토크, 빈 백 시험, 스트로크별 체질로 검증합니다. PINN 은 DEM 을 100 스트로크 돌리기엔 비용이 너무 커서, 몇 스트로크의 DEM 결과와 드문 체질 데이터를 물리 손실로 묶어 외삽하고 역보정하는 자리에 들어갑니다. 원래 계획이었던 하중 데이터에서 PINN 으로 입도분포를 예측하는 구조가 입력 데이터를 바로잡은 뒤 다시 살아나는 셈입니다.')

for sid, h in slides.items():
    open(os.path.join(OUT, sid + '.html'), 'w', encoding='utf-8').write(h)
print(len(slides))
