import time
import pandas as pd
import numpy as np
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("-f", "--file", type=str, default="", help="file to flip axes")
args = parser.parse_args()

csv_file_name = args.file
df_control = pd.read_csv(csv_file_name)

df_control['q_fl2'] = df_control['q_fl2'].apply(lambda x: x*-1)
df_control['q_fl3'] = df_control['q_fl3'].apply(lambda x: x*-1)
df_control['q_fr2'] = df_control['q_fr2'].apply(lambda x: x*-1)
df_control['q_fr3'] = df_control['q_fr3'].apply(lambda x: x*-1)
df_control['q_bl1'] = df_control['q_bl1'].apply(lambda x: x*-1)
df_control['q_br1'] = df_control['q_br1'].apply(lambda x: x*-1)

df_control['qd_fl2'] = df_control['qd_fl2'].apply(lambda x: x*-1)
df_control['qd_fl3'] = df_control['qd_fl3'].apply(lambda x: x*-1)
df_control['qd_fr2'] = df_control['qd_fr2'].apply(lambda x: x*-1)
df_control['qd_fr3'] = df_control['qd_fr3'].apply(lambda x: x*-1)
df_control['qd_bl1'] = df_control['qd_bl1'].apply(lambda x: x*-1)
df_control['qd_br1'] = df_control['qd_br1'].apply(lambda x: x*-1)

df_control['Tau_fl2'] = df_control['Tau_fl2'].apply(lambda x: x*-1)
df_control['Tau_fl3'] = df_control['Tau_fl3'].apply(lambda x: x*-1)
df_control['Tau_fr2'] = df_control['Tau_fr2'].apply(lambda x: x*-1)
df_control['Tau_fr3'] = df_control['Tau_fr3'].apply(lambda x: x*-1)
df_control['Tau_bl1'] = df_control['Tau_bl1'].apply(lambda x: x*-1)
df_control['Tau_br1'] = df_control['Tau_br1'].apply(lambda x: x*-1)

df_control.to_csv(csv_file_name)
