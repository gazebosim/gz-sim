# Produces two PNG graphs of EntityComponentManager::Each performance
# from the "each" example program.
#
# Output filenames:
#
#     matching-entity.png : Graph of matching entity count vs each duration.
#     nonmatching-entity.png : Graph of nonmatching entity count vs each duration.
#
# Usage:
#   gnuplot -e "filename='MY_FILENAME'" each.gp
#

set terminal png size 1920,1080 enhanced font "Roboto, 20"
set ylabel 'ns' tc lt 1
set xlabel 'Matching entity count'
set grid
set linetype 1 lw 2
set linetype 2 lw 2

set output "matching-entity.png"
set title "Matching Entity Count vs Avg Each() duration"
plot filename using 1:3 every 10 linetype 1 with linespoints title 'Cache', \
     filename using 1:4 every 10 linetype 2 with linespoints title 'Cacheless'

set xlabel 'Nonmatching entity count'
set output "nonmatching-entity.png"
set title "Nonmatching Entity Count vs Avg Each() duration"
plot filename using 2:3 every ::::9 linetype 1 with linespoints title 'Cache', \
     filename using 2:4 every ::::9 linetype 2 with linespoints title 'Cacheless'
