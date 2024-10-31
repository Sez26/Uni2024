import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

def arm_animation(L1, L2, x_a, x_b, y_a, y_b, ref, num_int, dt):

    fig = plt.figure(figsize=(5, 4))
    ax = fig.add_subplot(autoscale_on=False, xlim=(-1.5, (L1+L2+0.5)), ylim=(-(L1+L2), (L1+L2)))
    ax.set_aspect('equal')
    ax.grid()

    # plot perfect reference
    ax.plot(ref[:,0], ref[:,1], 'r-', linewidth = 1)

    line, = ax.plot([], [], 'o-', lw=2)
    trace, = ax.plot([], [], '.-', lw=1, ms=2)
    time_template = 'time = %.1fs'
    time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)

    def animate(i):
        thisx = [0, x_a[i], x_b[i]]
        thisy = [0, y_a[i], y_b[i]]

        history_x = x_b[:i]
        history_y = y_b[:i]

        line.set_data(thisx, thisy)
        trace.set_data(history_x, history_y)
        time_text.set_text(time_template % (i*dt))
        return line, trace, time_text

    ani = animation.FuncAnimation(
        fig, animate, num_int, interval=dt*num_int, blit=True)
    plt.show()