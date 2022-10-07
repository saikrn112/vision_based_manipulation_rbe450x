import pandas as pd
import matplotlib.pyplot as plt

col_names = ["x_blue","y_blue", "x_green","y_green", "x_pink","y_pink", "x_red","y_red"]
df = pd.read_csv('/root/vbm/data.csv',names=col_names)


def cvt(df):
    return df.head(70).to_numpy().tolist()

fig, axs = plt.subplots(1,2)

axs[0].plot(cvt(df['x_blue']),color='b',label='blue')
axs[0].plot(cvt(df['x_green']),color='g',label='green')
axs[0].plot(cvt(df['x_red']),color='r',label='red')
axs[0].plot(cvt(df['x_pink']),color='purple',label='purple')
axs[0].set_title('X vs time')
axs[0].legend()
axs[1].plot(cvt(df['y_blue']),color='b',label='blue')
axs[1].plot(cvt(df['y_green']),color='g',label='green')
axs[1].plot(cvt(df['y_red']),color='r',label='red')
axs[1].plot(cvt(df['y_pink']),color='purple',label='purple')
axs[1].legend()
axs[1].set_title('Y vs time')
plt.suptitle('Color Center\'s coordinates over time')
plt.savefig("x_y_time")

plt.figure()
plt.plot(cvt(df['x_blue']),cvt(df['y_blue']),color='blue',label='blue')
plt.plot(cvt(df['x_green']),cvt(df['y_green']),color='green',label='green')
plt.plot(cvt(df['x_pink']),cvt(df['y_pink']),color='red',label='red')
plt.plot(cvt(df['x_red']),cvt(df['y_red']),color='purple',label='purple')
plt.legend()
plt.suptitle('Color Center\'s trajectory')
plt.savefig("x_y_trajectory")
