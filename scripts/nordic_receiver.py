import datetime
from math import ceil
import random
from matplotlib import pyplot as plt
import serial
import re
import pandas

# fig = plt.figure()

# def append_data_to_figure(index, setting):
#     x.append(index)
#     y.append(convert_setting(setting[0], setting[1], setting[2]))
#     print(x, y)
#     fig.clf()
#     ax = fig.add_axes([0.1, 0.1, 0.8, 0.8])
#     ax.set_yticks([convert_setting(22, 16, 0), convert_setting(23, 0, 0), convert_setting(23, 16, 0), convert_setting(24, 0, 0), convert_setting(24, 16, 0)])
#     ax.set_yticklabels(['22.16.0', '23.0.0', '23.16.0', '24.0.0', '24.16.0'])
#     ax.scatter(x, y)

#     plt.pause(1)
#     plt.ioff()

# port_name = input("Please input {COM*}:")
port_name = "COM16"
port = serial.Serial(port_name, 115200)
coarse_code = 0
mid_code = 0
fine_code = 0
now_coarse_code = 0
now_mid_code = 0
now_fine_code = 0
count = 0
record = {}
result = []
first_setup = True
while True:
    data = port.readline()
    # print("Received: ", data)
    if len(data) >= 11:
        now_coarse_code = int(data[8])
        now_mid_code = int(data[9])
        now_fine_code = int(data[10])
        if now_coarse_code == 22 and now_mid_code == 30 and now_fine_code == 8:
            if record.get("22.30.9", None) != None:
                result.append(["22.30.9", record["22.30.9"]])
            record = {}
            continue
        if record.get(f"{now_coarse_code}.{now_mid_code}.{now_fine_code}", None) is None:
            record[f"{now_coarse_code}.{now_mid_code}.{now_fine_code}"] = 1
        else:
            record[f"{now_coarse_code}.{now_mid_code}.{now_fine_code}"] += 1
        # if first_setup:
        #     first_setup = False
        #     coarse_code = now_coarse_code
        #     mid_code = now_mid_code
        #     fine_code = now_fine_code
        # if now_coarse_code!=coarse_code or now_mid_code!=mid_code or now_fine_code!=fine_code:
        #     print(f"{coarse_code}.{mid_code}.{fine_code}", f" received {count} packets")
        #     coarse_code = now_coarse_code
        #     mid_code = now_mid_code
        #     fine_code = now_fine_code
        #     count = 0
        # count+=1
        # print([{k:record[k]}for k in record])
        # pandas.DataFrame([record]).to_csv("output.csv")
        pandas.DataFrame(result).to_csv("output.csv")
        # pandas.DataFrame([{k:record[k]}for k in record.keys()]).to_csv("output.csv")


# f = Figure()

# for i in range(300):
#     f.append_rx_setting(23, random.randint(0, 31), random.randint(0, 31))
#     f.append_tx_setting(23, random.randint(0, 31), random.randint(0, 31))
#     f.append_rc_2m_setting(23, random.randint(0, 31), random.randint(0, 31))
#     f.append_average_intermediate_frequency_count(500 + random.randint(-30, 30))
#     f.append_average_2M_RC_count(60000 + random.randint(-60, 60))
#     f.append_average_frequency_offset(0 + random.randint(-10, 10))
#     f.append_temperature(28 + random.randint(0, 20))
#     f.increase_x()
#     f.update()
#     plt.pause(0.1)
