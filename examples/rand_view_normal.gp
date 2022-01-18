set terminal jpeg
binwidth = 5
bin(x,width)=width*floor(x/width)

set tics out nomirror
set style fill transparent solid 0.5 border lt -1
set xrange [-1000:1000]
set xtics binwidth
set boxwidth binwidth
set yrange [0:2500]

plot "normal.data" u (bin($1,binwidth)):(1.0) smooth freq with boxes notitle
