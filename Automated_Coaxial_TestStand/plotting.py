import csv
#import pandas as pd
import numpy as np
from pandas import read_csv
import scipy as scipy
from scipy import stats
import matplotlib.pyplot as plt
from matplotlib import cm
import matplotlib as mpl
import pandas as pd

plt.rcParams['text.usetex'] = True
mpl.rcParams.update({'pgf.preamble': r'\usepackage{amsmath}'})
plt.style.use('my_style.txt')
fig_x=6.5; fig_y=3

fig_1 = plt.figure(figsize=(fig_x,fig_y))
ax_1 = fig_1.add_subplot()
ax_1.set_title("10in Diameter")
ax_1.set_xlabel(r"Motor Speed (RPM)", fontsize = 12)
ax_1.set_ylabel(r"Thrust $(kgf)$", fontsize = 12)


ax_1.legend([line_1, line_2, line_3, line_4, line_5, line_6, line_7, line_9, line_10, line_11, line_12],
            ['13.5$^{\\circ} $', '12$^{\\circ} $', '10.5$^{\\circ} $', '9$^{\\circ} $', '7.5$^{\\circ} $', 
'6$^{\\circ} $', '4.5$^{\\circ} $', '1.5$^{\\circ} $', '0$^{\\circ} $', '-1.5$^{\\circ} $', '-3$^{\\circ} $'])
ax_1.set_xlim(1.500, 6)
ax_1.set_ylim(0, None)
plt.xticks(fontsize = 10)
plt.yticks(fontsize = 10)
plt.savefig('Fit_Figures2/10in_Original_Thrust_Data2.png', dpi='figure', format='png',
        bbox_inches="tight", pad_inches=0.1,
        facecolor='auto', edgecolor='auto')