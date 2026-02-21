Must set log rate to high (SDLOG_PROFILE) 
for frequency response analysis post flight 
-> Can *technically* calculate gains from system response with this.

Trim Servo channels such that no input on manual -> no attitude change

Get pitch offset and cruise airspeed

If flight w manual is avail, do cruise, climb (max throttle), descend (zero throttle), hard right hard left, pitchup pitch down

then get parameters via log


# Fixed Wing Tuning:

Rate Controllers

Roll Rate:

Set PI to minimal value (0.005)
Set FF to 0.4 -> Increase until roll is good, then decrease by 20%
Set P to 0.06 -> Double until unstable, then halve
I to 0.01 -> Double until no sustained error

Pitch Rate: (same as Roll Rate)

Time Constants: 
FW_P_TC: increase -> softer response
FW_R_TC: increase -> softer response

TECS:
Stabilized -> Find trim for throttle and pitch for cruise @ set airspeed

Increase throttle until max airspeed -> FW_THR_MAX, FW_AIRSPD_MAX
FW_THR_MIN (don't really touch)

Stabilized + Full Throttle -> Pitch up until FW_AIRSPD_TRIM
FW_P_LIM_MAX, FW_T_CLIMB_MAX

Stabilized + Min Throttle -> Pitch down until FW_AIRSPD_MAX
FW_P_LIM_MIN, FW_T_SINK_MAX

Stabilized + Trim Throttle -> Pitch down until FW_AIRSPD_TRIM
FW_T_SINK_MIN

NPFG_PERIOD: don't touch!!

WEIGHT_BASE, WEIGHT_GROSS

# MC Tuning

Easy, not much to say for rate control 
