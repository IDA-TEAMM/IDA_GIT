import sys, math
sys.path.insert(0, "/root/ros2_ws/_scan_wt/son_kodv2/karar")
sys.path.insert(0, "/root/ros2_ws/_scan_wt/son_kodv2/karar/scripts")
from parkur2_orani import kosum, URETIM_K, URETIM_T
from prototype.mission.parkur_dunyasi import oku as parkuru_oku

dunya = parkuru_oku()
son_p1 = dunya.kapilar[-1]
merkez = ((son_p1[0][0]+son_p1[1][0])/2.0+3.0, (son_p1[0][1]+son_p1[1][1])/2.0)

denemeler = [
    ("normal", merkez, 0.0),
    ("yanal +3m", (merkez[0], merkez[1]+3.0), 0.0),
    ("yanal -3m", (merkez[0], merkez[1]-3.0), 0.0),
    ("aci +20", merkez, math.radians(20)),
    ("aci -20", merkez, math.radians(-20)),
]
for ad, bas, yon in denemeler:
    r = kosum(dunya.kapilar_p2, dunya.engeller, baslangic=bas, yon0_rad=yon,
              gn5_gercek=dunya.gn5, sure=300, mppi_k=URETIM_K, mppi_t=URETIM_T)
    print(f"{ad:12s} kapi={r['gecilen']:2d}/{r['toplam_kapi']} son_kapi={r['son_kapi_gecildi']} "
          f"GN5={r['gn5_varildi']} carpma={r['carpma']} pay={r['en_kucuk_pay']:.2f}m "
          f"sure={r['sure']:.0f}s tamamlandi={r['parkur2_tamamlandi']}", flush=True)
