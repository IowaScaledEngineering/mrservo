v 20130925 2
N 42000 54100 44000 54100 4
C 47500 52800 1 0 0 gnd-1.sym
C 46200 54900 1 0 0 lm7805-1.sym
{
T 47800 56200 5 10 0 0 0 0 1
device=7805
T 47600 55900 5 10 1 1 0 6 1
refdes=U2
T 46200 54900 5 10 0 0 0 0 1
footprint=TO220
}
C 45400 54900 1 270 0 capacitor-1.sym
{
T 46100 54700 5 10 0 1 270 0 1
device=CAPACITOR
T 45700 54600 5 10 1 1 0 0 1
refdes=C1
T 46300 54700 5 10 0 0 270 0 1
symversion=0.1
T 45700 54300 5 10 1 1 0 2 1
value=10uF
T 45400 54900 5 10 0 0 0 0 1
footprint=0805
T 45700 54100 5 10 1 1 0 2 1
comment=25V
}
N 44000 53300 48200 53300 4
N 47000 53300 47000 54900 4
N 45600 54900 45600 55500 4
N 45600 54000 45600 53300 4
N 47800 55500 53600 55500 4
C 53100 54600 1 0 0 gnd-1.sym
N 53200 54900 53200 55900 4
N 53200 55900 53600 55900 4
N 47600 53100 47600 53300 4
N 48200 54900 48200 55500 4
N 48200 54000 48200 53300 4
T 54400 55400 9 10 1 0 0 0 1
5V
T 54400 55800 9 10 1 0 0 0 1
GND
T 54400 55000 9 10 1 0 0 0 1
SERVO CNTL
N 42400 51300 44400 51300 4
C 47400 49500 1 0 0 PIC10F200-1.sym
{
T 50300 50850 5 10 1 1 0 6 1
refdes=U1
T 47700 50840 5 10 0 0 0 0 1
device=PIC10F200
T 47700 51050 5 10 0 0 0 0 1
footprint=SOT26
T 47700 51650 5 10 0 0 0 0 1
symversion=1.0
}
N 51400 54300 51400 55100 4
N 51400 55100 53600 55100 4
N 45400 51300 51200 51300 4
N 51200 50500 51200 51300 4
N 50600 50500 54500 50500 4
N 50600 50100 50800 50100 4
N 50800 50100 50800 55500 4
C 46500 48800 1 0 0 gnd-1.sym
N 47400 50100 46600 50100 4
N 46600 50100 46600 49100 4
N 52800 44800 52800 55500 4
N 52800 50100 54500 50100 4
N 54500 49700 50600 49700 4
N 47000 42000 47000 49700 4
N 47000 49300 54500 49300 4
N 47400 49700 47000 49700 4
C 53300 48400 1 0 0 gnd-1.sym
N 53400 48700 53400 48900 4
N 53400 48900 54500 48900 4
T 55300 50000 9 10 1 0 0 0 1
5V
T 55300 48800 9 10 1 0 0 0 1
GND
T 55300 49600 9 10 1 0 0 0 1
RELAY1
T 55300 49200 9 10 1 0 0 0 1
RELAY2 / ICSPCLK
N 47000 52000 47000 50500 4
C 42000 53500 1 0 1 termblk3-1.sym
{
T 41000 54150 5 10 0 0 0 6 1
device=HEADER3
T 41600 54800 5 10 1 1 0 6 1
refdes=J1
T 42000 53500 5 10 0 0 0 0 1
footprint=TERMBLK3_3p5MM
}
N 42000 54500 44000 54500 4
N 42400 51300 42400 53700 4
N 42400 53700 42000 53700 4
T 41200 54400 9 10 1 0 0 6 1
12V
T 41200 54000 9 10 1 0 0 6 1
GND
T 41200 53600 9 10 1 0 0 6 1
CNTL
C 54500 48700 1 0 0 header6-1.sym
{
T 55500 49350 5 10 0 0 0 0 1
device=HEADER3
T 54900 51200 5 10 1 1 0 0 1
refdes=J3
T 54500 48700 5 10 0 0 0 0 1
footprint=JUMPER6
}
T 55300 50400 9 10 1 0 0 0 1
MCLR
T 55300 50800 9 10 1 0 0 0 1
ICSPDAT
N 47000 50500 47400 50500 4
N 51400 53400 51400 52000 4
N 47000 52000 53200 52000 4
N 53200 52000 53200 50900 4
N 53200 50900 54500 50900 4
N 44000 54100 44000 53300 4
N 46200 55500 45200 55500 4
N 44300 55500 44000 55500 4
N 44000 55500 44000 54500 4
N 49600 55300 49600 55500 4
N 49600 54400 49600 54200 4
C 49500 52800 1 0 0 gnd-1.sym
N 49600 53100 49600 53300 4
N 50600 48500 51000 48500 4
N 49700 48500 49500 48500 4
C 48100 48000 1 0 0 gnd-1.sym
N 48600 48500 48200 48500 4
N 48200 48500 48200 48300 4
N 51000 42800 51000 49700 4
C 45400 51900 1 90 1 mosfet-with-diode-1.sym
{
T 44900 51000 5 10 0 0 90 6 1
device=NPN_TRANSISTOR
T 45200 51100 5 10 1 1 180 6 1
refdes=Q1
T 44400 50700 5 10 1 1 0 0 1
footprint=SOT23_MOS
T 45000 51800 5 10 1 1 0 0 1
device=BSS138
}
N 44900 51900 44900 52400 4
N 44900 52400 50800 52400 4
C 40000 40000 0 0 0 title-bordered-C.sym
C 56200 45900 1 0 0 termblk3-1.sym
{
T 57200 46550 5 10 0 0 0 0 1
device=HEADER3
T 56500 47150 5 10 1 1 0 0 1
refdes=J5
T 56200 45900 5 10 0 0 0 6 1
footprint=TERMBLK3_3p5MM
}
N 52800 47500 58200 47500 4
N 51400 42800 51000 42800 4
N 47000 42000 53800 42000 4
N 53800 42000 53800 42800 4
C 53100 44300 1 0 0 relay-DPDT-1.sym
{
T 53400 47150 5 10 1 1 0 0 1
refdes=K1
T 53100 44300 5 10 0 0 0 0 1
footprint=RELAY-DPDT
}
N 52800 44800 53100 44800 4
N 56200 46900 54200 46900 4
N 56200 46100 54200 46100 4
C 54600 44700 1 0 0 termblk3-1.sym
{
T 55600 45350 5 10 0 0 0 0 1
device=HEADER3
T 54900 44450 5 10 1 1 0 0 1
refdes=J4
T 54600 44700 5 10 0 0 0 6 1
footprint=TERMBLK3_3p5MM
}
N 54600 45700 54200 45700 4
N 54600 45300 54200 45300 4
N 54600 44900 54200 44900 4
C 58500 43800 1 0 0 relay-DPDT-1.sym
{
T 58800 46650 5 10 1 1 0 0 1
refdes=K2
T 58500 43800 5 10 0 0 0 0 1
footprint=RELAY-DPDT
}
N 58200 44300 58500 44300 4
N 60000 44800 59600 44800 4
N 59600 44400 60300 44400 4
N 54200 46500 54900 46500 4
N 54900 46500 54900 48100 4
N 54900 48100 60300 48100 4
N 60300 44400 60300 48100 4
N 60000 44800 60000 47800 4
N 60000 47800 55600 47800 4
N 55600 47800 55600 46500 4
N 55600 46500 56200 46500 4
C 52300 41300 1 0 1 gnd-1.sym
C 52800 41300 1 0 1 gnd-1.sym
T 55400 45650 9 10 1 0 0 0 1
1NOR
T 55400 45250 9 10 1 0 0 0 1
1COM
T 55400 44850 9 10 1 0 0 0 1
1REV
T 57000 46850 9 10 1 0 0 0 1
2NOR
T 57000 46450 9 10 1 0 0 0 1
2COM
T 57000 46050 9 10 1 0 0 0 1
2REV
C 51400 42200 1 0 0 drdc3105e6-1.sym
{
T 52950 43400 5 10 1 1 0 0 1
device=DRDC3105E6
T 53250 43150 5 10 1 1 0 0 1
refdes=U3
T 53000 42200 5 10 1 1 0 0 1
footprint=SOT26
}
N 53800 42800 53500 42800 4
N 52200 41600 52200 42200 4
N 52700 41600 52700 42200 4
N 52200 43500 52200 44400 4
N 52200 44400 53100 44400 4
N 58500 43900 52700 43900 4
N 52700 43900 52700 43500 4
N 58200 47500 58200 44300 4
C 49800 54400 1 90 0 led-3.sym
{
T 50050 54350 5 10 1 1 90 0 1
device=GREEN LED
T 49250 54850 5 10 1 1 90 0 1
refdes=D1
T 49800 54400 5 10 0 0 0 0 1
footprint=0805
}
C 49700 48300 1 0 0 led-3.sym
{
T 49750 48050 5 10 1 1 0 0 1
device=RED LED
T 50150 48850 5 10 1 1 0 0 1
refdes=D2
T 49700 48300 5 10 0 0 270 0 1
footprint=0805
}
C 44300 55200 1 0 0 schottky-diode-1.sym
{
T 44622 55872 5 10 0 0 0 0 1
device=DIODE
T 44700 55800 5 10 1 1 0 3 1
refdes=D3
T 45141 55632 5 10 1 1 0 0 1
footprint=SOD123T
T 44800 55200 5 10 1 1 0 5 1
device=CDBM140
}
N 59600 45600 60300 45600 4
N 59600 46000 60000 46000 4
C 54900 46400 1 0 0 SolderJumperOpen-3.sym
{
T 55250 46650 5 10 1 1 0 3 1
refdes=JP1
T 54900 48000 5 10 0 0 0 0 1
footprint=SolderJumperSmall
T 54900 48200 5 10 0 0 0 0 1
device=SolderJumper
}
C 48000 54900 1 270 0 capacitor-1.sym
{
T 48700 54700 5 10 0 1 270 0 1
device=CAPACITOR
T 48300 54600 5 10 1 1 0 0 1
refdes=C2
T 48900 54700 5 10 0 0 270 0 1
symversion=0.1
T 48300 54300 5 10 1 1 0 2 1
value=1uF
T 48000 54900 5 10 0 0 0 0 1
footprint=0805
T 48300 54100 5 10 1 1 0 2 1
comment=16V
}
C 49500 52900 1 270 1 res-pack4-1.sym
{
T 49500 52900 5 10 0 0 180 6 1
slot=1
T 49400 54000 5 10 0 1 180 1 1
footprint=RPACK4-1206
T 49750 53700 5 10 1 1 90 5 1
value=1k
T 49450 53700 5 10 1 1 90 3 1
refdes=R1
}
C 51300 53000 1 270 1 res-pack4-1.sym
{
T 51300 53000 5 10 0 0 180 6 1
slot=4
T 51200 54100 5 10 0 1 180 1 1
footprint=RPACK4-1206
T 51550 53800 5 10 1 1 90 5 1
value=1k
T 51250 53800 5 10 1 1 90 3 1
refdes=R1
}
C 48200 48600 1 180 1 res-pack4-1.sym
{
T 48200 48600 5 10 0 0 90 6 1
slot=2
T 49300 48700 5 10 0 1 90 1 1
footprint=RPACK4-1206
T 49000 48350 5 10 1 1 0 5 1
value=1k
T 49000 48650 5 10 1 1 0 3 1
refdes=R1
}
C 53600 56100 1 180 1 header3-1.sym
{
T 54600 55450 5 10 0 0 180 6 1
device=HEADER3
T 54100 54800 5 10 1 1 180 3 1
refdes=J2
T 53600 56100 5 10 0 0 180 6 1
footprint=JUMPER3-OFFSET
}
