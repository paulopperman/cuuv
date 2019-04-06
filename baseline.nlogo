;; This code models an artifical potential function navigation system
;; for an autonomous vehicle.
;;
;; This software is authored by Team 1.

__includes [
  "experiment_loader.nls"
  "measurement_functions.nls"
  "environment_setup.nls"
  "./model_source_files/threat_uuv_procedures_v2.nls"
  "./model_source_files/dvl_spoofer_procedures.nls"
  "./model_source_files/bubble_curtain_procedures.nls"
]

;; set global variables
globals [
  environment-folder  ; the folder containing environment setup files
  end-reached  ; boolean if the last waypoint has been reached
  patch-dim  ; length of a patch in meters
  tick-dim  ; duration of a tick in seconds (s/tick)
  threat-uuv-speed  ; converted uuv-speed
  max-turn  ; turning radius converted to degrees/tick
  forward_min_range  ; converted forward sonar minimum range
  forward_max_range  ; converted forward sonar max range
  side_min_range  ; converted side sonar min range
  side_max_range  ; converted side sonar max range
  current-speed  ; converted current speed
  nav-velocity-std  ; converted noise standard deviation
  max-nav-error  ; metric for maximum nav error
  experiment-number  ; iterator for behaviorspace
]


;; create agent types



;; define field parameters for patches
patches-own [
  potential
  behavior_x  ; x components of the behavior map vector
  behavior_y  ; y components of the behavior map vector
]



to convert-parameters
  ;; code to convert front panel real world parameters into model scale values
  set patch-dim 5  ;; patches are 5m
  set tick-dim 1  ;; ticks are 1s

  ; convert speed
  set threat-uuv-speed ((uuv-speed / patch-dim) * tick-dim)
  show threat-uuv-speed

  ; calculate turn angle
  set max-turn (2 * asin( uuv-speed / (2 * turn-radius )))
  show max-turn

  ; convert nav error
  set nav-velocity-std (nav-velocity-std-cm * .01 * patch-dim)

  ; convert sonar ranges
  set forward_min_range (forward_low_range / patch-dim)
  set forward_max_range (forward_hi_range / patch-dim)
  set side_min_range (side_low_range / patch-dim)
  set side_max_range (side_hi_range / patch-dim)

  ; convert current speed
  set current-speed ((drift-speed / patch-dim) * tick-dim)

end


to setup-environment
  ;;; procedure to manually setup the world
  clear-all
  ; set up the world

;  let experiment-file "./experiments/mine_error_straight_run/mine_error_straight_run_1.txt"
;  read-experiment experiment-number experiment-file


  ;resize-world 0 499 0 999
  ;show "world resized"
  ;set-patch-size 0.2
  ;show "patches sized"

  set environment-folder "./environments/baseline/"  ; the folder containing environment setup files
  show environment-folder

  setup-world word environment-folder "world_dims.txt"

  load-mission-waypoints word environment-folder "mission_waypoints.txt"
  show "mission_waypoints.txt loaded"

  ;load-vector-data word environment-folder "mission_leg_0.txt"  ; load the initial vector field
  load-vector-data-v2 environment-folder

  ;; initialize obstacles
  place-objects-from-file word environment-folder "obstacles.txt"
  ;place-random-objects 20 200 10 300 989

  ;; place the minefield
  lay-mines-from-file word environment-folder "minefield.txt"
  ;lay-random-mines nmines minefield_minx minefield_miny minefield_maxx minefield_maxy

  setup-uuv

  reset-ticks
end


to go

  let ping (ticks mod floor sonar_ping_rate) = 0

  ask uuvs [
    update-mission-segment   ; Checks current position, and updates mission segment and associated vector profiles if necessay
    current-drift
    if ping [
      detect-sonar-contacts
      ;show "ping"
    ]      ; Looks around UUV, gets all contacts (mines, obstacles) and determines what kind of contact it is.  Do this every ping_rate ticks
    navigate-threat-uuv  ; move the threat uuv
    check-collisions ; see if it hits anything
  ]
  ;show navigation-error
  if navigation-error > max-nav-error [ set max-nav-error navigation-error ]
  if ping [ spoof-dvl ]
  if end-reached [ stop ]

  tick  ;; next simulation step
end

to current-drift
  let drift_x current-speed * sin current-heading
  let drift_y current-speed * cos current-heading
  ask ( turtle-set uuvs self-position-fixes )  [  ; anything that isn't tied down drifts with the current
    carefully [ setxy (xcor + drift_x) (ycor + drift_y) ] [ ]  ; do this carefully to avoid out of bounds errors at world edge
  ]
end
@#$#@#$#@
GRAPHICS-WINDOW
619
13
1019
422
-1
-1
0.2
1
10
1
1
1
0
0
0
1
0
399
0
399
1
1
1
ticks
30.0

BUTTON
292
833
428
866
NIL
setup-environment
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

BUTTON
329
923
392
956
NIL
go
T
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

SLIDER
397
63
569
96
max-obs-dist
max-obs-dist
1
100
21.5
0.1
1
NIL
HORIZONTAL

SLIDER
397
106
569
139
obs-influence
obs-influence
0
5
3.0
0.1
1
NIL
HORIZONTAL

SLIDER
15
66
187
99
turn-radius
turn-radius
1
30
14.3
1
1
m
HORIZONTAL

SLIDER
14
102
186
135
uuv-speed
uuv-speed
.1
5
1.2
.01
1
m/s
HORIZONTAL

BUTTON
442
833
530
866
NIL
setup-uuv
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

BUTTON
443
875
524
908
hide links
hide-link
NIL
1
T
LINK
NIL
NIL
NIL
NIL
1

SLIDER
12
497
184
530
forward_angle
forward_angle
0
100
65.0
1
1
deg
HORIZONTAL

SLIDER
14
319
186
352
side_angle
side_angle
30
160
25.0
1
1
deg
HORIZONTAL

PLOT
1635
36
2054
399
Percent Mines Detected
Time (ticks)
% Mines Detected
0.0
32000.0
0.0
100.0
true
false
"" ""
PENS
"default" 1.0 0 -16777216 true "" "plot 100 * (count mine-contacts / count mines)"

TEXTBOX
30
17
180
51
UUV Hydrodynamic Parameters
14
0.0
1

TEXTBOX
20
155
170
189
SONAR Sensor Parameters
14
0.0
1

SLIDER
202
319
374
352
side_low_range
side_low_range
0
300
5.0
.1
1
m
HORIZONTAL

SLIDER
385
319
572
352
side_hi_range
side_hi_range
50
800
684.0
.1
1
NIL
HORIZONTAL

SLIDER
200
500
375
533
forward_low_range
forward_low_range
0
20
1.0
.1
1
m
HORIZONTAL

SLIDER
383
499
570
532
forward_hi_range
forward_hi_range
10
600
203.1
.1
1
m
HORIZONTAL

TEXTBOX
20
293
170
311
Side Scan Sonar
11
0.0
1

TEXTBOX
22
473
172
491
Forward Look Sonar
11
0.0
1

SLIDER
14
363
186
396
side_p_detect
side_p_detect
.3
1
0.0
.01
1
NIL
HORIZONTAL

SLIDER
18
200
214
233
sonar_ping_rate
sonar_ping_rate
1
100
3.0
1
1
ticks/ping
HORIZONTAL

SLIDER
12
542
184
575
forward_p_detect
forward_p_detect
0
1
0.0
.01
1
NIL
HORIZONTAL

SWITCH
12
586
192
619
forward_detect_mines
forward_detect_mines
1
1
-1000

SWITCH
11
410
190
443
side_detect_obstacles
side_detect_obstacles
1
1
-1000

TEXTBOX
406
10
556
44
UUV Command and Control Parameters
14
0.0
1

BUTTON
444
917
526
950
NIL
show-link
NIL
1
T
LINK
NIL
NIL
NIL
NIL
1

SLIDER
201
66
386
99
nav-bearing-std
nav-bearing-std
0
5
0.0
.001
1
deg/s
HORIZONTAL

SLIDER
200
104
393
137
nav-velocity-std-cm
nav-velocity-std-cm
0
10
0.0
.01
1
cm/s
HORIZONTAL

PLOT
1635
416
2053
718
Position Error
ticks
error
0.0
1.0
0.0
1.0
true
false
"" ""
PENS
"default" 1.0 0 -16777216 true "" "plot navigation-error"

SLIDER
325
709
497
742
current-heading
current-heading
0
359
143.0
1
1
deg
HORIZONTAL

SLIDER
325
747
497
780
drift-speed
drift-speed
0
2
0.2
.01
1
m/s
HORIZONTAL

SLIDER
15
711
187
744
nmines
nmines
10
100
100.0
1
1
NIL
HORIZONTAL

SLIDER
14
750
186
783
minefield_minx
minefield_minx
min-pxcor
max-pxcor
100.0
1
1
NIL
HORIZONTAL

SLIDER
13
788
185
821
minefield_miny
minefield_miny
min-pycor
max-pycor
50.0
1
1
NIL
HORIZONTAL

SLIDER
13
825
185
858
minefield_maxx
minefield_maxx
min-pxcor
max-pxcor
400.0
1
1
NIL
HORIZONTAL

SLIDER
14
864
186
897
minefield_maxy
minefield_maxy
min-pycor
max-pycor
950.0
1
1
NIL
HORIZONTAL

BUTTON
444
962
571
995
lay random mines
lay-random-mines nmines minefield_minx minefield_miny minefield_maxx minefield_maxy
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

TEXTBOX
231
23
381
41
Navigation Errors
14
0.0
1

TEXTBOX
17
685
167
703
Minefield Settings
14
0.0
1

BUTTON
287
877
429
910
NIL
convert-parameters
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

PLOT
1638
739
2054
1015
plot 1
NIL
NIL
0.0
10.0
0.0
10.0
true
false
"" ""
PENS
"pen-2" 1.0 0 -7500403 true "" "plot (first [heading] of uuvs)"
"pen-1" 1.0 0 -2674135 true "" "plot (first [heading] of self-position-fixes)"

PLOT
1175
142
1607
491
plot 2
NIL
NIL
0.0
5.0
0.0
10.0
true
false
"" ""
PENS
"default" 1.0 0 -16777216 true "" "plot objective-mine-speed"
"pen-1" 1.0 0 -7500403 true "" "plot objective-mine-position"

PLOT
1198
557
1568
858
Contact SNR
NIL
NIL
0.0
100.0
0.0
20.0
true
false
"" ""
PENS
"default" 1.0 1 -16777216 true "" "histogram [contact-snr] of sonar-contacts"

SLIDER
227
200
399
233
source-level
source-level
1
160
117.5
.1
1
dB
HORIZONTAL

SLIDER
229
240
461
273
classification-threshold-mean
classification-threshold-mean
0
80
3.0
.1
1
dB
HORIZONTAL

SLIDER
329
636
539
669
signal-factor
signal-factor
0
100
0.0
.1
1
dB
HORIZONTAL

SLIDER
370
385
542
418
noise-floor
noise-floor
-100
100
0.0
1
1
NIL
HORIZONTAL

SLIDER
241
281
460
314
classification-threshold-std
classification-threshold-std
0
10
4.9
.1
1
dB
HORIZONTAL

BUTTON
632
924
753
957
NIL
deploy-spoofers
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

@#$#@#$#@
## WHAT IS IT?

This code implements an artificial potential field model for autonomous vehicle behavior.

## HOW IT WORKS

The potential value of the patches defines the potential field.  The autonomous vehicle moves downhill to minimize its potential.

## HOW TO USE IT

1. Initialize the model by clicking the setup button.
2. Click the edit-map button to place rocks.  Click the button again when done.
3. Click go to increment the model by 1 time step.  Repeat to progress through the simulation.

## THINGS TO NOTICE

(suggested things for the user to notice while running the model)

## THINGS TO TRY

(suggested things for the user to try to do (move sliders, switches, etc.) with the model)

## EXTENDING THE MODEL

The potential field is very simple for demonstration purposes.  More field configuration options are needed.

## NETLOGO FEATURES

(interesting or unusual features of NetLogo that the model uses, particularly in the Code tab; or where workarounds were needed for missing features)

## RELATED MODELS

(models in the NetLogo Models Library and elsewhere which are of related interest)

## CREDITS AND REFERENCES

Barisic, Matko, Nikola Miskovic, and Zoran Vukic. 2009. Heuristic Parameter Tuning Procedures for a Virtual Potential Based AUV Trajectory Planner. IFAC Proceedings Volumes (IFAC-PapersOnline). Vol. 42. IFAC. https://doi.org/10.3182/20090916-3-BR-3001.0043.

Healey, A. 2006. “Guidance Laws, Obstacle Avoidance and Artificial Potential Functions.” In Advances in Unmanned Marine Vehicles, edited by G. N. Roberts and R. Sutton, 43–66. London: Institution of Engineering and Technology. https://doi.org/10.1049/PBCE069E_ch3.

Rimon, Elon, and Daniel E Koditschek. 1992. “Exact Robot Navigation Using Artificial Potential Functions.” IEEE Transactions on Robotics and Automation 8 (5): 501–18. http://ieeexplore.ieee.org/xpls/abs_all.jsp?arnumber=163777.
@#$#@#$#@
default
true
0
Polygon -7500403 true true 150 5 40 250 150 205 260 250

airplane
true
0
Polygon -7500403 true true 150 0 135 15 120 60 120 105 15 165 15 195 120 180 135 240 105 270 120 285 150 270 180 285 210 270 165 240 180 180 285 195 285 165 180 105 180 60 165 15

arrow
true
0
Polygon -7500403 true true 150 0 0 150 105 150 105 293 195 293 195 150 300 150

box
false
0
Polygon -7500403 true true 150 285 285 225 285 75 150 135
Polygon -7500403 true true 150 135 15 75 150 15 285 75
Polygon -7500403 true true 15 75 15 225 150 285 150 135
Line -16777216 false 150 285 150 135
Line -16777216 false 150 135 15 75
Line -16777216 false 150 135 285 75

bug
true
0
Circle -7500403 true true 96 182 108
Circle -7500403 true true 110 127 80
Circle -7500403 true true 110 75 80
Line -7500403 true 150 100 80 30
Line -7500403 true 150 100 220 30

butterfly
true
0
Polygon -7500403 true true 150 165 209 199 225 225 225 255 195 270 165 255 150 240
Polygon -7500403 true true 150 165 89 198 75 225 75 255 105 270 135 255 150 240
Polygon -7500403 true true 139 148 100 105 55 90 25 90 10 105 10 135 25 180 40 195 85 194 139 163
Polygon -7500403 true true 162 150 200 105 245 90 275 90 290 105 290 135 275 180 260 195 215 195 162 165
Polygon -16777216 true false 150 255 135 225 120 150 135 120 150 105 165 120 180 150 165 225
Circle -16777216 true false 135 90 30
Line -16777216 false 150 105 195 60
Line -16777216 false 150 105 105 60

car
false
0
Polygon -7500403 true true 300 180 279 164 261 144 240 135 226 132 213 106 203 84 185 63 159 50 135 50 75 60 0 150 0 165 0 225 300 225 300 180
Circle -16777216 true false 180 180 90
Circle -16777216 true false 30 180 90
Polygon -16777216 true false 162 80 132 78 134 135 209 135 194 105 189 96 180 89
Circle -7500403 true true 47 195 58
Circle -7500403 true true 195 195 58

circle
false
0
Circle -7500403 true true 0 0 300

circle 2
false
0
Circle -7500403 true true 0 0 300
Circle -16777216 true false 30 30 240

cow
false
0
Polygon -7500403 true true 200 193 197 249 179 249 177 196 166 187 140 189 93 191 78 179 72 211 49 209 48 181 37 149 25 120 25 89 45 72 103 84 179 75 198 76 252 64 272 81 293 103 285 121 255 121 242 118 224 167
Polygon -7500403 true true 73 210 86 251 62 249 48 208
Polygon -7500403 true true 25 114 16 195 9 204 23 213 25 200 39 123

cylinder
false
0
Circle -7500403 true true 0 0 300

dot
false
0
Circle -7500403 true true 90 90 120

face happy
false
0
Circle -7500403 true true 8 8 285
Circle -16777216 true false 60 75 60
Circle -16777216 true false 180 75 60
Polygon -16777216 true false 150 255 90 239 62 213 47 191 67 179 90 203 109 218 150 225 192 218 210 203 227 181 251 194 236 217 212 240

face neutral
false
0
Circle -7500403 true true 8 7 285
Circle -16777216 true false 60 75 60
Circle -16777216 true false 180 75 60
Rectangle -16777216 true false 60 195 240 225

face sad
false
0
Circle -7500403 true true 8 8 285
Circle -16777216 true false 60 75 60
Circle -16777216 true false 180 75 60
Polygon -16777216 true false 150 168 90 184 62 210 47 232 67 244 90 220 109 205 150 198 192 205 210 220 227 242 251 229 236 206 212 183

fish
false
0
Polygon -1 true false 44 131 21 87 15 86 0 120 15 150 0 180 13 214 20 212 45 166
Polygon -1 true false 135 195 119 235 95 218 76 210 46 204 60 165
Polygon -1 true false 75 45 83 77 71 103 86 114 166 78 135 60
Polygon -7500403 true true 30 136 151 77 226 81 280 119 292 146 292 160 287 170 270 195 195 210 151 212 30 166
Circle -16777216 true false 215 106 30

flag
false
0
Rectangle -7500403 true true 60 15 75 300
Polygon -7500403 true true 90 150 270 90 90 30
Line -7500403 true 75 135 90 135
Line -7500403 true 75 45 90 45

flower
false
0
Polygon -10899396 true false 135 120 165 165 180 210 180 240 150 300 165 300 195 240 195 195 165 135
Circle -7500403 true true 85 132 38
Circle -7500403 true true 130 147 38
Circle -7500403 true true 192 85 38
Circle -7500403 true true 85 40 38
Circle -7500403 true true 177 40 38
Circle -7500403 true true 177 132 38
Circle -7500403 true true 70 85 38
Circle -7500403 true true 130 25 38
Circle -7500403 true true 96 51 108
Circle -16777216 true false 113 68 74
Polygon -10899396 true false 189 233 219 188 249 173 279 188 234 218
Polygon -10899396 true false 180 255 150 210 105 210 75 240 135 240

house
false
0
Rectangle -7500403 true true 45 120 255 285
Rectangle -16777216 true false 120 210 180 285
Polygon -7500403 true true 15 120 150 15 285 120
Line -16777216 false 30 120 270 120

leaf
false
0
Polygon -7500403 true true 150 210 135 195 120 210 60 210 30 195 60 180 60 165 15 135 30 120 15 105 40 104 45 90 60 90 90 105 105 120 120 120 105 60 120 60 135 30 150 15 165 30 180 60 195 60 180 120 195 120 210 105 240 90 255 90 263 104 285 105 270 120 285 135 240 165 240 180 270 195 240 210 180 210 165 195
Polygon -7500403 true true 135 195 135 240 120 255 105 255 105 285 135 285 165 240 165 195

line
true
0
Line -7500403 true 150 0 150 300

line half
true
0
Line -7500403 true 150 0 150 150

pentagon
false
0
Polygon -7500403 true true 150 15 15 120 60 285 240 285 285 120

person
false
0
Circle -7500403 true true 110 5 80
Polygon -7500403 true true 105 90 120 195 90 285 105 300 135 300 150 225 165 300 195 300 210 285 180 195 195 90
Rectangle -7500403 true true 127 79 172 94
Polygon -7500403 true true 195 90 240 150 225 180 165 105
Polygon -7500403 true true 105 90 60 150 75 180 135 105

plant
false
0
Rectangle -7500403 true true 135 90 165 300
Polygon -7500403 true true 135 255 90 210 45 195 75 255 135 285
Polygon -7500403 true true 165 255 210 210 255 195 225 255 165 285
Polygon -7500403 true true 135 180 90 135 45 120 75 180 135 210
Polygon -7500403 true true 165 180 165 210 225 180 255 120 210 135
Polygon -7500403 true true 135 105 90 60 45 45 75 105 135 135
Polygon -7500403 true true 165 105 165 135 225 105 255 45 210 60
Polygon -7500403 true true 135 90 120 45 150 15 180 45 165 90

rock
false
0
Circle -7500403 true true 63 108 85
Circle -7500403 true true 96 81 108
Circle -7500403 true true 146 131 67
Rectangle -7500403 true true 75 165 195 195

sheep
false
15
Circle -1 true true 203 65 88
Circle -1 true true 70 65 162
Circle -1 true true 150 105 120
Polygon -7500403 true false 218 120 240 165 255 165 278 120
Circle -7500403 true false 214 72 67
Rectangle -1 true true 164 223 179 298
Polygon -1 true true 45 285 30 285 30 240 15 195 45 210
Circle -1 true true 3 83 150
Rectangle -1 true true 65 221 80 296
Polygon -1 true true 195 285 210 285 210 240 240 210 195 210
Polygon -7500403 true false 276 85 285 105 302 99 294 83
Polygon -7500403 true false 219 85 210 105 193 99 201 83

square
false
0
Rectangle -7500403 true true 30 30 270 270

square 2
false
0
Rectangle -7500403 true true 30 30 270 270
Rectangle -16777216 true false 60 60 240 240

star
false
0
Polygon -7500403 true true 151 1 185 108 298 108 207 175 242 282 151 216 59 282 94 175 3 108 116 108

target
false
0
Circle -7500403 true true 0 0 300
Circle -16777216 true false 30 30 240
Circle -7500403 true true 60 60 180
Circle -16777216 true false 90 90 120
Circle -7500403 true true 120 120 60

tree
false
0
Circle -7500403 true true 118 3 94
Rectangle -6459832 true false 120 195 180 300
Circle -7500403 true true 65 21 108
Circle -7500403 true true 116 41 127
Circle -7500403 true true 45 90 120
Circle -7500403 true true 104 74 152

triangle
false
0
Polygon -7500403 true true 150 30 15 255 285 255

triangle 2
false
0
Polygon -7500403 true true 150 30 15 255 285 255
Polygon -16777216 true false 151 99 225 223 75 224

truck
false
0
Rectangle -7500403 true true 4 45 195 187
Polygon -7500403 true true 296 193 296 150 259 134 244 104 208 104 207 194
Rectangle -1 true false 195 60 195 105
Polygon -16777216 true false 238 112 252 141 219 141 218 112
Circle -16777216 true false 234 174 42
Rectangle -7500403 true true 181 185 214 194
Circle -16777216 true false 144 174 42
Circle -16777216 true false 24 174 42
Circle -7500403 false true 24 174 42
Circle -7500403 false true 144 174 42
Circle -7500403 false true 234 174 42

turtle
true
0
Polygon -10899396 true false 215 204 240 233 246 254 228 266 215 252 193 210
Polygon -10899396 true false 195 90 225 75 245 75 260 89 269 108 261 124 240 105 225 105 210 105
Polygon -10899396 true false 105 90 75 75 55 75 40 89 31 108 39 124 60 105 75 105 90 105
Polygon -10899396 true false 132 85 134 64 107 51 108 17 150 2 192 18 192 52 169 65 172 87
Polygon -10899396 true false 85 204 60 233 54 254 72 266 85 252 107 210
Polygon -7500403 true true 119 75 179 75 209 101 224 135 220 225 175 261 128 261 81 224 74 135 88 99

wheel
false
0
Circle -7500403 true true 3 3 294
Circle -16777216 true false 30 30 240
Line -7500403 true 150 285 150 15
Line -7500403 true 15 150 285 150
Circle -7500403 true true 120 120 60
Line -7500403 true 216 40 79 269
Line -7500403 true 40 84 269 221
Line -7500403 true 40 216 269 79
Line -7500403 true 84 40 221 269

wolf
false
0
Polygon -16777216 true false 253 133 245 131 245 133
Polygon -7500403 true true 2 194 13 197 30 191 38 193 38 205 20 226 20 257 27 265 38 266 40 260 31 253 31 230 60 206 68 198 75 209 66 228 65 243 82 261 84 268 100 267 103 261 77 239 79 231 100 207 98 196 119 201 143 202 160 195 166 210 172 213 173 238 167 251 160 248 154 265 169 264 178 247 186 240 198 260 200 271 217 271 219 262 207 258 195 230 192 198 210 184 227 164 242 144 259 145 284 151 277 141 293 140 299 134 297 127 273 119 270 105
Polygon -7500403 true true -1 195 14 180 36 166 40 153 53 140 82 131 134 133 159 126 188 115 227 108 236 102 238 98 268 86 269 92 281 87 269 103 269 113

x
false
0
Polygon -7500403 true true 270 75 225 30 30 225 75 270
Polygon -7500403 true true 30 75 75 30 270 225 225 270
@#$#@#$#@
NetLogo 6.0.4
@#$#@#$#@
@#$#@#$#@
@#$#@#$#@
<experiments>
  <experiment name="baseline vulnerability study" repetitions="10" sequentialRunOrder="false" runMetricsEveryStep="false">
    <setup>cp
cd
ct
clear-all-plots

set number-of-collisions 0
set max-nav-error 0
let experiment-file "./experiments/v2_NOLH.txt"
read-experiment-v2 experiment-number experiment-file
set environment-folder "./environments/baseline/"
setup-world word environment-folder "world_dims.txt"
load-mission-waypoints word environment-folder "mission_waypoints.txt"
load-vector-data-v2 environment-folder
place-objects-from-file word environment-folder "obstacles.txt"
lay-mines-from-file word environment-folder "minefield.txt"
setup-uuv
reset-ticks</setup>
    <go>go</go>
    <timeLimit steps="10000"/>
    <exitCondition>end-reached</exitCondition>
    <metric>number-of-collisions</metric>
    <metric>max-nav-error</metric>
    <metric>count mine-contacts</metric>
    <metric>marker-mean</metric>
    <metric>marker-std-dev</metric>
    <steppedValueSet variable="experiment-number" first="0" step="1" last="128"/>
  </experiment>
  <experiment name="threat baseline run" repetitions="1" sequentialRunOrder="false" runMetricsEveryStep="false">
    <setup>cp
cd
ct
clear-all-plots

set number-of-collisions 0
set max-nav-error 0
let experiment-file "./experiments/baseline_NOLH.txt"
read-baseline-experiment experiment-number experiment-file
set environment-folder "./environments/baseline/"
setup-world word environment-folder "world_dims.txt"
load-mission-waypoints word environment-folder "mission_waypoints.txt"
load-vector-data-v2 environment-folder
place-objects-from-file word environment-folder "obstacles.txt"
lay-mines-from-file word environment-folder "minefield.txt"
setup-uuv
reset-ticks</setup>
    <go>go</go>
    <timeLimit steps="30000"/>
    <exitCondition>end-reached</exitCondition>
    <metric>number-of-collisions</metric>
    <metric>max-nav-error</metric>
    <metric>count mine-contacts</metric>
    <metric>marker-mean</metric>
    <metric>marker-std-dev</metric>
    <steppedValueSet variable="experiment-number" first="0" step="1" last="64"/>
    <enumeratedValueSet variable="random-seed">
      <value value="1"/>
      <value value="3"/>
      <value value="5"/>
      <value value="8"/>
      <value value="22"/>
      <value value="153"/>
      <value value="98341"/>
      <value value="88"/>
      <value value="99"/>
      <value value="15"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="random spoofers" repetitions="1" sequentialRunOrder="false" runMetricsEveryStep="false">
    <setup>cp
cd
ct
clear-all-plots

set number-of-collisions 0
set max-nav-error 0
let experiment-file "./experiments/spoofer_NOLH.txt"
read-spoofer-experiment experiment-number experiment-file
set environment-folder "./environments/baseline/"
setup-world word environment-folder "world_dims.txt"
load-mission-waypoints word environment-folder "mission_waypoints.txt"
load-vector-data-v2 environment-folder
place-objects-from-file word environment-folder "obstacles.txt"
lay-mines-from-file word environment-folder "minefield.txt"
setup-uuv
deploy-spoofers
reset-ticks</setup>
    <go>go</go>
    <timeLimit steps="30000"/>
    <exitCondition>end-reached</exitCondition>
    <metric>number-of-collisions</metric>
    <metric>max-nav-error</metric>
    <metric>count mine-contacts</metric>
    <metric>marker-mean</metric>
    <metric>marker-std-dev</metric>
    <steppedValueSet variable="experiment-number" first="0" step="1" last="128"/>
    <enumeratedValueSet variable="random-seed">
      <value value="1"/>
      <value value="3"/>
      <value value="5"/>
      <value value="8"/>
      <value value="22"/>
      <value value="153"/>
      <value value="98341"/>
      <value value="88"/>
      <value value="99"/>
      <value value="15"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="random spoofers noseed" repetitions="10" sequentialRunOrder="false" runMetricsEveryStep="false">
    <setup>cp
cd
ct
clear-all-plots

set number-of-collisions 0
set max-nav-error 0
let experiment-file "./experiments/spoofer_NOLH.txt"
read-spoofer-experiment experiment-number experiment-file
set environment-folder "./environments/baseline/"
setup-world word environment-folder "world_dims.txt"
load-mission-waypoints word environment-folder "mission_waypoints.txt"
load-vector-data-v2 environment-folder
place-objects-from-file word environment-folder "obstacles.txt"
lay-mines-from-file word environment-folder "minefield.txt"
setup-uuv
deploy-spoofers
reset-ticks</setup>
    <go>go</go>
    <timeLimit steps="30000"/>
    <exitCondition>end-reached</exitCondition>
    <metric>number-of-collisions</metric>
    <metric>max-nav-error</metric>
    <metric>count mine-contacts</metric>
    <metric>marker-mean</metric>
    <metric>marker-std-dev</metric>
    <steppedValueSet variable="experiment-number" first="0" step="1" last="128"/>
  </experiment>
  <experiment name="random big spoofers" repetitions="1" sequentialRunOrder="false" runMetricsEveryStep="false">
    <setup>cp
cd
ct
clear-all-plots

set number-of-collisions 0
set max-nav-error 0
let experiment-file "./experiments/big_spoofer_NOLH.txt"
read-spoofer-experiment experiment-number experiment-file
set environment-folder "./environments/baseline/"
setup-world word environment-folder "world_dims.txt"
load-mission-waypoints word environment-folder "mission_waypoints.txt"
load-vector-data-v2 environment-folder
place-objects-from-file word environment-folder "obstacles.txt"
lay-mines-from-file word environment-folder "minefield.txt"
setup-uuv
deploy-spoofers
reset-ticks</setup>
    <go>go</go>
    <timeLimit steps="30000"/>
    <exitCondition>end-reached</exitCondition>
    <metric>number-of-collisions</metric>
    <metric>max-nav-error</metric>
    <metric>count mine-contacts</metric>
    <metric>marker-mean</metric>
    <metric>marker-std-dev</metric>
    <steppedValueSet variable="experiment-number" first="0" step="1" last="128"/>
    <enumeratedValueSet variable="random-seed">
      <value value="1"/>
      <value value="3"/>
      <value value="5"/>
      <value value="8"/>
      <value value="22"/>
      <value value="153"/>
      <value value="98341"/>
      <value value="88"/>
      <value value="99"/>
      <value value="15"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="random big spoofers alt" repetitions="1" sequentialRunOrder="false" runMetricsEveryStep="false">
    <setup>cp
cd
ct
clear-all-plots

set number-of-collisions 0
set max-nav-error 0
let experiment-file "./experiments/big_spoofer_NOLH.txt"
read-spoofer-alt-experiment experiment-number experiment-file
set environment-folder "./environments/baseline/"
setup-world word environment-folder "world_dims.txt"
load-mission-waypoints word environment-folder "mission_waypoints.txt"
load-vector-data-v2 environment-folder
place-objects-from-file word environment-folder "obstacles.txt"
lay-mines-from-file word environment-folder "minefield.txt"
setup-uuv
deploy-spoofers
reset-ticks</setup>
    <go>go</go>
    <timeLimit steps="30000"/>
    <exitCondition>end-reached</exitCondition>
    <metric>number-of-collisions</metric>
    <metric>max-nav-error</metric>
    <metric>count mine-contacts</metric>
    <metric>marker-mean</metric>
    <metric>marker-std-dev</metric>
    <steppedValueSet variable="experiment-number" first="0" step="1" last="128"/>
    <enumeratedValueSet variable="random-seed">
      <value value="1"/>
      <value value="3"/>
      <value value="5"/>
      <value value="8"/>
      <value value="22"/>
      <value value="153"/>
      <value value="98341"/>
      <value value="88"/>
      <value value="99"/>
      <value value="15"/>
    </enumeratedValueSet>
  </experiment>
</experiments>
@#$#@#$#@
@#$#@#$#@
default
0.0
-0.2 0 0.0 1.0
0.0 1 1.0 0.0
0.2 0 0.0 1.0
link direction
true
0
Line -7500403 true 150 150 90 180
Line -7500403 true 150 150 210 180
@#$#@#$#@
0
@#$#@#$#@
