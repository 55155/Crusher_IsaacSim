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
NUMH = '<div style="display:flex;gap:10px"><p style="font-size:18px;color:#7F7F7F;width:118px">식 번호 2002 · 2009</p></div>'
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
                  '<p style="font-size:19px;color:#7F7F7F;text-align:center">굵은 테두리 = 기존 DEM 에 추가<br>2002 식 (9)–(11) = 해석해, DEM 검증 기준 (0.3 %p)</p>',
                  w='width:100%'), w='flex:1', h='align-items:stretch;padding:38px 24px 16px;'), gap=28, grow=True),
    '제가 공부한 Tavares 그룹의 반복충격 손상 모델입니다. 식 번호는 2002 논문과 2009 논문을 같이 적었습니다. 식 (1)부터 (4)는 한 번의 충격에서 손상이 쌓이고 강성이 떨어지는 과정이고, 2002 식 (9)부터 (11), 2009 식 (5)와 (6)은 그 결과 파쇄 비에너지가 충격마다 줄어드는 과정입니다. 2009 식 (13)은 깨질 때 파편 입도입니다. 오른쪽은 이 식들이 DEM 한 스텝의 어디에 들어가는지입니다.')

