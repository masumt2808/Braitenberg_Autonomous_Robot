import csv
import math
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

gains     = [0.3, 0.8, 1.5]
colors    = ['#4C72B0', '#55A868', '#C44E52']

avg_speed     = []
avg_turn_rate = []
avg_obs_dist  = []

for g in gains:
    path = f'/home/masum/braitenberg_ws/gain_log_{g}.csv'
    speeds, turns, dists = [], [], []
    try:
        with open(path) as f:
            reader = csv.DictReader(f)
            for row in reader:
                speeds.append(float(row['v']))
                # only count turning when obstacle present
                if int(row['obstacle']) == 1:
                    turns.append(abs(float(row['w'])))
                    dists.append(float(row['obs_dist']))
        avg_speed.append(sum(speeds)/len(speeds) if speeds else 0)
        avg_turn_rate.append(sum(turns)/len(turns) if turns else 0)
        avg_obs_dist.append(sum(dists)/len(dists) if dists else 0)
        print(f'GAIN={g}: {len(speeds)} samples, '
              f'avg_v={avg_speed[-1]:.3f}, '
              f'avg_w={avg_turn_rate[-1]:.3f}, '
              f'avg_obs={avg_obs_dist[-1]:.3f}')
    except FileNotFoundError:
        print(f'Missing log file for GAIN={g} — run the robot first')
        avg_speed.append(0)
        avg_turn_rate.append(0)
        avg_obs_dist.append(0)

fig, axes = plt.subplots(1, 3, figsize=(13, 5))
fig.suptitle('Effect of BRAITENBERG_GAIN on Robot Behavior\n(data from live simulation)',
             fontsize=13, fontweight='bold')

def bar_plot(ax, values, title, ylabel, ylim, extra_fn=None):
    bars = ax.bar([str(g) for g in gains], values, color=colors, width=0.4)
    ax.set_title(title, fontsize=11)
    ax.set_xlabel('BRAITENBERG_GAIN')
    ax.set_ylabel(ylabel)
    ax.set_ylim(0, ylim)
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.grid(axis='y', alpha=0.3)
    if extra_fn:
        extra_fn(ax)
    for bar, val in zip(bars, values):
        ax.text(bar.get_x() + bar.get_width()/2, val + ylim*0.02,
                f'{val:.3f}', ha='center', fontsize=10, fontweight='bold')

bar_plot(axes[0], avg_speed,     'Avg Forward Speed',              'Speed (m/s)',        0.22)
bar_plot(axes[1], avg_turn_rate, 'Avg Turn Rate\n(obstacle only)', 'Angular vel (rad/s)',2.5)
bar_plot(axes[2], avg_obs_dist,  'Avg Obstacle Distance\n(when detected)', 'Distance (m)', 1.0,
         lambda ax: ax.axhline(y=0.8, color='gray', linestyle='--',
                               linewidth=1, label='DANGER_DIST'))

axes[2].legend(fontsize=8)

patches = [mpatches.Patch(color=colors[i], label=f'GAIN = {g}')
           for i, g in enumerate(gains)]
fig.legend(handles=patches, loc='lower center', ncol=3,
           fontsize=10, bbox_to_anchor=(0.5, -0.04))

plt.tight_layout()
out = '/home/masum/braitenberg_ws/gain_analysis.png'
plt.savefig(out, dpi=150, bbox_inches='tight')
plt.show()
print(f'saved → {out}')
