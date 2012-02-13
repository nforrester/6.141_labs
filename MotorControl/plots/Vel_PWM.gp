#!/usr/bin/gnuplot -persist 

# Gnuplot script file for plotting angular velocity data

# This script can be run with the command: ./scriptname filename
# to produce an output file 

# A data file should have the format: "X.XXX X.XXX X.XXX X.XXX ... " per line, 
# where the columns are

# 1: Time since start of experiment (s)
# 2: PWM left motor -255..255
# 3: PWM right motor -255..255
# 4: Encoder Angular velocity left motor (rad/sec)
# 5: Encoder Angular velocity right motor (rad/sec)
# 6: Desired angular velocity left motor (rad/sec)
# 7: Desired angular velocity right motor (rad/sec)

# plot-independent parameters
set time                        # date/time in lower-left corner
set autoscale                   # scale axes automatically
set xtic auto                   # set xtics automatically
set ytic auto                   # set ytics automatically
set size 1.0, 1.0               # set the plot size

# plot-dependent parameters
set title "Velocity-PWM Relation for Unloaded Motor" # plot title
set xlabel "Commanded PWM (percent)"                 # x-axis label
set ylabel "Motor Angular Velocity (radians/second)" # y-axis label
set terminal postscript enhanced mono lw 2 "Helvetica" 14

set out "plot.ps"       # plot filename
# commanded PWM on X axis, encoder-inferred motor velocity on Y axis    
# plot "data.txt" using 3:5 title 'Motor Velocity, as measured by encoder' with lines
# N.B.: using points, because the time-series output by the logger will 
# not generally include a monotone set of PWM values from -255 to 255
plot "data.txt" using 3:5 title 'Motor Velocity, as measured by encoder' with points
