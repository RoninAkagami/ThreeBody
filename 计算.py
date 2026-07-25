# save_sim_png.py
import requests



def nuy(a,b,c,d,e):
    params = {
        "y2": a,
        "vx2": b,
        "vy2": c,
        "vx3": d,
        "vy3": e,
    }

    url = "http://127.0.0.1:60001/"

    response = requests.get(url, params=params, timeout=None)
    response.raise_for_status()

    with open(str(params["y2"])+"c"+str(params["vx2"])+"c"+str(params["vy2"])+"c"+str(params["vx3"])+"c"+str(params["vy3"])+".png", "wb") as f:
        f.write(response.content)

    print("saved: sim_output_percentile_inverted.png")


qwe=0
while True:
    nuy(100.1,0,qwe,1,0)
    qwe-=1
    if qwe==100:
        break
