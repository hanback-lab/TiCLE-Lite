from ws2812 import WS2812Matrix
import utime, urandom

W=16; H=16; N=20
m=WS2812Matrix([(3,1)])
m.clear()

def rnd(n): return urandom.getrandbits(16)%n
def frand(a,b): return a+(urandom.getrandbits(16)/65535.0)*(b-a)

parts=[ [rnd(W), frand(-H, H-1), frand(0.08,0.22), (200,180,80)] for _ in range(N) ]

while True:
    for y in range(H):
        for x in range(W):
            m[x,y].value = (0,0,0)
    for p in parts:
        x=int(p[0]); y=int(p[1])
        if 0<=y<H: m[x,y].value=p[3]
        p[1]+=p[2]
        if p[1]>=H:
            p[0]=rnd(W)
            p[1]=frand(-4.0,-0.5)
            p[2]=frand(0.08,0.22)
    m.update()
    utime.sleep_ms(35)