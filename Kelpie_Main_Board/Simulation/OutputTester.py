V1 = 4.2
v2 = 4.1
v3 = 4.1335
v4 = 4.15

V2 = 2*v2 - V1
V3 = 3*v3 - V2 - V1
V4 = 4*v4 - v3 - v2 - V1

print(f"V1: {V1}, V2: {V2}, V3: {V3}, V4: {V4}")