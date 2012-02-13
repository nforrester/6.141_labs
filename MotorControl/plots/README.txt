
This file contains instructions for using the "makeplot" script
to launch gnuplot to produce various useful plots.

The gnuplot program can't take filenames as parameters.  Instead,
all of the scripts expect data.txt as input, and produce plot.ps.
There is a single wrapper "makeplot" that takes the gnuplot file
and applies it to the data file to produce a named postscript plot.

Invoke makeplot as specified below to produce the 6 specified plots:

./makeplot Vel_PWM.gp Vel_PWM.txt plot1.ps

./makeplot FFVel_Unloaded.gp dataLog2.txt plot2.ps

./makeplot FFVel_Loaded.gp dataLog3.txt plot3.ps

./makeplot PVel_VariedGain.gp dataLog4.txt plot4.ps

./makeplot PVel_FixedGain.gp dataLog5.txt plot5.ps

./makeplot DError_Loaded.gp dataLog6.txt plot6.ps


