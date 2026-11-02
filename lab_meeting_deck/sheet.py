import sys,os
from PIL import Image
d=sys.argv[1]
fs=sorted([f for f in os.listdir(d) if f.lower().endswith('.png')],key=lambda f:int(''.join(c for c in f if c.isdigit())))
for k in range(0,len(fs),6):
    im=Image.new('RGB',(1280*2,720*3),'white')
    for i,f in enumerate(fs[k:k+6]):
        im.paste(Image.open(os.path.join(d,f)).convert("RGB").resize((1280,720)),((i%2)*1280,(i//2)*720))
    im.resize((1280,1080)).save(os.path.join(d,f'..\sheet{k//6}.png'))
