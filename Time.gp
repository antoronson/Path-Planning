set terminal png enhanced
set terminal postscript enhanced
set terminal postscript eps enhanced
set term postscript enhanced color
set output "colorindex.ps"
set key box lw 3
set key height 1
set key left
set border lw 4
set grid
set lmargin at screen 0.19
set style line 1 lt 0 lc rgb "red" lw 3
set style line 2 lt 2 lw 3
set style line 3 lt 3 lw 3
plot "start.dat" using 1:2  title "Home" with linespoints ls 1 pt 1 ps 2 , "end.dat" using 1:2  title "Target" with linespoints ls 2 pt 3 ps 2, "block.dat" using 1:2  title "Pre Occupied" with points pt 2 ps 2, "path.dat" using 1:2  title "Path" with linespoints ls 6 pt 4 ps 2 
set title "Time Comparison" font "arial,24"
set xtic font "arial,15"
set ytic font "arial,15"
set xrange [0:100]
set yrange [0:100]
set xlabel "N sums"  font "arial,18"
set ylabel "Time (ms)" font "arial,18"
set terminal png font arial 20 size 1024,768
set output "Path.pn"
replot

