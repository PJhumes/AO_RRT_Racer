import pandas as pd
df = pd.read_csv('track_info/tracks/IMS.csv', comment='#', names=['x_m', 'y_m', 'w_tr_right_m', 'w_tr_left_m'])

df['x_m'] = df['x_m'] * .2
df['y_m'] = df['y_m'] * .2

df.to_csv('track_info/tracks/IMS_Shrunk.csv', index=False)
