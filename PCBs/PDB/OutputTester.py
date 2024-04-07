


def calc_v_divider(v1, v2, v3, v4):
    v2_divider = (1/2) * (v1+v2)
    v3_divider = (1 / 3) * (v1 + v2 + v3)
    v4_divider = (1 / 4) * (v1 + v2 + v3 + v4)
    return v1, v2_divider, v3_divider, v4_divider

def calc_cells(v1, v2, v3, v4):
    V1 = v1
    V2 = 2 * v2 - V1
    V3 = 3 * v3 - V2 - V1
    V4 = 4 * v4 - V3 - V2 - V1
    return round(V1, 3), round(V2, 3), round(V3, 3), round(V4, 3)


v1_gt = 4.2
v2_gt = 4.2
v3_gt = 4.2
v4_gt = 4.2

divider_out = calc_v_divider(v1_gt, v2_gt, v3_gt, v4_gt)
V1, V2, V3, V4 = calc_cells(*divider_out)
print(f"V1: {V1}\nV2: {V2}\nV3: {V3}\nV4: {V4}")