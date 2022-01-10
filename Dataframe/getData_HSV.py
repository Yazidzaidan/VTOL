#contoh penarikan data dari excel

import pandas as pd

df = pd.read_excel('hsv_ELP.xlsx')
df.info()
print("Data HSV ELP")
print(df)
h_min = df['h'].min()
s_min = df['s'].min()
v_min = df['v'].min()
h_max = df['h'].max()
s_max = df['s'].max()
v_max = df['v'].max()
print("lower HSV :", h_min, s_min, v_min)
print("upper HSV :", h_max, s_max, v_max)

