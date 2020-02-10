# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2018 Bitcraze AB
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.
"""
Simple example that connects to one crazyflie (check the address at the top
and update it to your crazyflie address) and uses the high level commander
to send setpoints and trajectory to fly a figure 8.

This example is intended to work with any positioning system (including LPS).
It aims at documenting how to set the Crazyflie in position control mode
and how to send setpoints using the high level commander.
"""
import threading
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.mem import MemoryElement
from cflib.crazyflie.mem import Poly4D
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
import matplotlib.pyplot as plt
import drawnow as drawnow

# URI to the Crazyflie to connect to
uri = 'radio://0/80/2M/E7E7E7E7E2'

# The trajectory to fly
# See https://github.com/whoenig/uav_trajectories for a tool to generate
# trajectories

# Duration,x^0,x^1,x^2,x^3,x^4,x^5,x^6,x^7,y^0,y^1,y^2,y^3,y^4,y^5,y^6,y^7,z^0,z^1,z^2,z^3,z^4,z^5,z^6,z^7,yaw^0,yaw^1,yaw^2,yaw^3,yaw^4,yaw^5,yaw^6,yaw^7

# Duration,x^0,x^1,x^2,x^3,x^4,x^5,x^6,x^7,y^0,y^1,y^2,y^3,y^4,y^5,y^6,y^7,z^0,z^1,z^2,z^3,z^4,z^5,z^6,z^7,yaw^0,yaw^1,yaw^2,yaw^3,yaw^4,yaw^5,yaw^6,yaw^7
""" 


figure8 = [
    [1.050000, 0.000000, -0.000000, 0.000000, -0.000000, 0.830443, -0.276140, -0.384219, 0.180493, -0.000000, 0.000000, -0.000000, 0.000000, -1.356107, 0.688430, 0.587426, -0.329106, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.710000, 0.396058, 0.918033, 0.128965, -0.773546, 0.339704, 0.034310, -0.026417, -0.030049, -0.445604, -0.684403, 0.888433, 1.493630, -1.361618, -0.139316, 0.158875, 0.095799, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.620000, 0.922409, 0.405715, -0.582968, -0.092188, -0.114670, 0.101046, 0.075834, -0.037926, -0.291165, 0.967514, 0.421451, -1.086348, 0.545211, 0.030109, -0.050046, -0.068177, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.700000, 0.923174, -0.431533, -0.682975, 0.177173, 0.319468, -0.043852, -0.111269, 0.023166, 0.289869, 0.724722, -0.512011, -0.209623, -0.218710, 0.108797, 0.128756, -0.055461, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.560000, 0.405364, -0.834716, 0.158939, 0.288175, -0.373738, -0.054995, 0.036090, 0.078627, 0.450742, -0.385534, -0.954089, 0.128288, 0.442620, 0.055630, -0.060142, -0.076163, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.560000, 0.001062, -0.646270, -0.012560, -0.324065, 0.125327, 0.119738, 0.034567, -0.063130, 0.001593, -1.031457, 0.015159, 0.820816, -0.152665, -0.130729, -0.045679, 0.080444, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.700000, -0.402804, -0.820508, -0.132914, 0.236278, 0.235164, -0.053551, -0.088687, 0.031253, -0.449354, -0.411507, 0.902946, 0.185335, -0.239125, -0.041696, 0.016857, 0.016709, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.620000, -0.921641, -0.464596, 0.661875, 0.286582, -0.228921, -0.051987, 0.004669, 0.038463, -0.292459, 0.777682, 0.565788, -0.432472, -0.060568, -0.082048, -0.009439, 0.041158, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.710000, -0.923935, 0.447832, 0.627381, -0.259808, -0.042325, -0.032258, 0.001420, 0.005294, 0.288570, 0.873350, -0.515586, -0.730207, -0.026023, 0.288755, 0.215678, -0.148061, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [1.053185, -0.398611, 0.850510, -0.144007, -0.485368, -0.079781, 0.176330, 0.234482, -0.153567, 0.447039, -0.532729, -0.855023, 0.878509, 0.775168, -0.391051, -0.713519, 0.391628, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
]"""
"""
figure8 = [
    [10.0, 2.5, 0, 0, 0, -0.016175, 0.004557, -0.00042475, 0.0000131, 2.5, 0, 0, 0, -0.023375, 0.0056775, -0.00046525,
     0.000012875, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # noqa
    [10.0, 2.7, 0, -0.3375, 0, 0.00816954076, -0.000354490418, -0.0000319224015, 0.00000174919237, 0, 1.35, 0, -0.05625,
     -0.00152854672, 0.00135872453, -0.000103841612, 0.00000234155309, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    # noqa
    [10.0, 0.765887901, 1.29454777, -0.0957359876, -0.0539394904, 0.000851629227, 0.0012023584, -0.00010863142,
     0.00000274155182, -2.58909554, 0.38294395, 0.323636943, -0.0159559979, -0.00826756185, 0.000725348235,
     0.00000115522722, -0.00000101313295, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # noqa
    [10.0, -2.26549313, 0.7344285, 0.283186641, -0.0306011875, -0.00768639074, 0.00103661764, -0.0000297068507,
     -0.000000193843202, -1.468857, -1.13274656, 0.183607125, 0.0471977735, -0.0031618426, -0.000947216794, 0.000104497,
     -0.0000029163281, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # noqa
    [10.0, -2.05115736, -0.877888584, 0.256394671, 0.036578691, -0.00521230602, -0.000614259952, 0.000091778,
     -0.0000028515238, 1.75577717, -1.02557868, -0.219472146, 0.0427324451, 0.00647377148, -0.00126272741,
     0.0000581284677, -0.000000641371051, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # noqa
    [10.0, 1.10182157, -1.23247609, -0.137727696, 0.0513531704, 0.00472932251, -0.00138510228, 0.0000817747468,
     -0.00000142389574, 2.46495218, 0.550910783, -0.308119022, -0.022954616, 0.00683457094, 0.000230840763,
     -0.0000715193039, 0.00000255246267, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # noqa
    [10.0, 2.67624759, 0.178674863, -0.334530949, -0.00744478594, 0.00789536594, -0.00017154233, -0.0000453851932,
     0.00000204371304, -0.357349725, 1.3381238, 0.0446687157, -0.0557551582, -0.00259635283, 0.001393689,
     -0.0000987031118, 0.00000208944533, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # noqa
    [10.0, 0.416478915, 1.33384269, -0.0520598643, -0.0555767789, 0.00805216792, -0.000123669613, -0.0000339399814,
     0.0000015313123, -2.66768539, 0.208239457, 0.333460673, -0.00867664406, -0.0159533, 0.00274483665, -0.000178712661,
     0.00000421778632, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # noqa
    [10.0, 2.5, 0, 0, 0, 0, 0, 0, 0, 2.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # noqa
]"""
"""
figure8=[[  1.0,         2.7,            0,             0,              0,        -1.6875,            4.725,          -4.3875,               1.35,          2.7,            0,             0,              0,        -1.6875,            4.725,           -4.3875,               1.35,            0,            0,            0,            0,          67.5,          -161.0,           133.5,             -38.0, 0, 0, 0, 0, 0, 0, 0, 0],  # noqa
[  1.0,         2.7,            0,       -0.3375,              0,  0.00703099564, 0.00000102032368, -0.0000601307202,   0.00000103186163,          2.7,            0,       -0.3375,              0,  0.00703099564, 0.00000102032368,  -0.0000601307202,   0.00000103186163,          2.0,            0,         -0.5,            0,  0.0416445487, 0.0000895330492,  -0.00152558631,   0.0000938104292, 0, 0, 0, 0, 0, 0, 0, 0],  # noqa
[  1.0,  2.36947292, -0.647224477,  -0.296184115,   0.0269676865,  0.00617030672,  -0.000336304066, -0.0000526316541,   0.00000284294916,   2.36947292, -0.647224477,  -0.296184115,   0.0269676865,  0.00617030672,  -0.000336304066,  -0.0000526316541,   0.00000284294916,   1.54030231, -0.841470985, -0.270151153,  0.140245164,   0.022509507,  -0.00699721596, -0.000779642296,    0.000195182412, 0, 0, 0, 0, 0, 0, 0, 0],  # noqa
[ 10.0,  1.45881623,  -1.13598583,  -0.182352028,   0.0473327429,   0.0309791059,    -0.0080883436,   0.000662891137,   -0.0000182853344,   1.45881623,  -1.13598583,  -0.182352028,   0.0473327429,   0.0309791059,    -0.0080883436,    0.000662891137,   -0.0000182853344,  0.583853163, -0.909297427,  0.208073418,  0.151549571, -0.0407350692,    0.0028699191, -0.000010214613, -0.00000332643898, 0, 0, 0, 0, 0, 0, 0, 0],  # noqa
[ 10.0, 0.190990445,  -1.34661823, -0.0238738056,    0.056109093, -0.00121735856,  -0.000553280602,  0.0000330375687, -0.000000328244353,  0.190990445,  -1.34661823, -0.0238738056,    0.056109093, -0.00841735856,   0.000567219398, -0.00000746243127, -0.000000553244353, 0.0100075034, -0.141120008,  0.494996248, 0.0235200013, -0.0526202515,   0.00984449131,   -0.0007164717,    0.000018760665, 0, 0, 0, 0, 0, 0, 0, 0],  # noqa
[ 10.0,         2.7,            0,       -0.3375,              0,  0.00816954076,  -0.000354490418, -0.0000319224015,   0.00000174919237,            0,         1.35,             0,       -0.05625, -0.00152854672,    0.00135872453,   -0.000103841612,   0.00000234155309,          1.0,            0,            0,            0,             0,               0,               0,                 0, 0, 0, 0, 0, 0, 0, 0, 0],  # noqa
[ 10.0, 0.765887901,   1.29454777, -0.0957359876,  -0.0539394904, 0.000851629227,     0.0012023584,   -0.00010863142,   0.00000274155182,  -2.58909554,   0.38294395,   0.323636943,  -0.0159559979, -0.00826756185,   0.000725348235,  0.00000115522722,  -0.00000101313295,          1.0,            0,            0,            0,             0,               0,               0,                 0, 0, 0, 0, 0, 0, 0, 0, 0],  # noqa
[ 10.0, -2.26549313,    0.7344285,   0.283186641,  -0.0306011875, -0.00768639074,    0.00103661764, -0.0000297068507, -0.000000193843202,    -1.468857,  -1.13274656,   0.183607125,   0.0471977735,  -0.0031618426,  -0.000947216794,       0.000104497,   -0.0000029163281,          1.0,            0,            0,            0,             0,               0,               0,                 0, 0, 0, 0, 0, 0, 0, 0, 0],  # noqa
[ 10.0, -2.05115736, -0.877888584,   0.256394671,    0.036578691, -0.00521230602,  -0.000614259952,      0.000091778,   -0.0000028515238,   1.75577717,  -1.02557868,  -0.219472146,   0.0427324451,  0.00647377148,   -0.00126272741,   0.0000581284677, -0.000000641371051,          1.0,            0,            0,            0,             0,               0,               0,                 0, 0, 0, 0, 0, 0, 0, 0, 0],  # noqa
[ 10.0,  1.10182157,  -1.23247609,  -0.137727696,   0.0513531704,  0.00472932251,   -0.00138510228,  0.0000817747468,  -0.00000142389574,   2.46495218,  0.550910783,  -0.308119022,   -0.022954616,  0.00683457094,   0.000230840763,  -0.0000715193039,   0.00000255246267,          1.0,            0,            0,            0,             0,               0,               0,                 0, 0, 0, 0, 0, 0, 0, 0, 0],  # noqa
[ 10.0,  2.67624759,  0.178674863,  -0.334530949, -0.00744478594,  0.00789536594,   -0.00017154233, -0.0000453851932,   0.00000204371304, -0.357349725,    1.3381238,  0.0446687157,  -0.0557551582, -0.00259635283,      0.001393689,  -0.0000987031118,   0.00000208944533,          1.0,            0,            0,            0,             0,               0,               0,                 0, 0, 0, 0, 0, 0, 0, 0, 0],  # noqa
[ 10.0, 0.416478915,   1.33384269, -0.0520598643,  -0.0555767789,  0.00875216792,  -0.000291669613, -0.0000199399814,    0.0000011313123,  -2.66768539,  0.208239457,   0.333460673, -0.00867664406,     -0.0152533,    0.00257683665,   -0.000164712661,   0.00000381778632,          1.0,            0,            0,            0,       -0.0035,         0.00084,        -0.00007,          0.000002, 0, 0, 0, 0, 0, 0, 0, 0],  # noqa
 ]"""
"""
figure1=[[  1.0,        2.7,            0,             0,               0,             16.25,             -38.5,              31.75,              -9.0,        2.7,             0,             0,              0,              16.25,              -38.5,               31.75,              -9.0,           0,             0,             0,             0,           41.5,           -99.4,             82.7,              -23.6, 0, 0, 0, 0, 0, 0, 0, 0],  # noqa
[  1.0,        3.2,            0,         -0.25,               0,      0.0208222743,   0.0000447665247,    -0.000762793154,   0.0000469052146,        3.2,             0,         -0.25,              0,       0.0208222743,    0.0000447665247,     -0.000762793154,   0.0000469052146,         1.2,             0,          -0.1,             0,  0.00832890974, 0.0000179066098,  -0.000305117262,    0.0000187620858, 0, 0, 0, 0, 0, 0, 0, 0],  # noqa
[  1.0, 2.97015115, -0.420735492,  -0.135075576,    0.0701225821,      0.0112547535,    -0.00349860798,    -0.000389821148,   0.0000975912058, 2.97015115,  -0.420735492,  -0.135075576,   0.0701225821,       0.0112547535,     -0.00349860798,     -0.000389821148,   0.0000975912058,  1.10806046,  -0.168294197, -0.0540302306,  0.0280490328,  0.00450190141,  -0.00139944319,  -0.000155928459,    0.0000390364823, 0, 0, 0, 0, 0, 0, 0, 0],  # noqa
[ 10.0, 2.49192658, -0.454648713,   0.104036709,    0.0757747856,     -0.0203675346,     0.00143495955,   -0.0000051073065, -0.00000166321949, 2.49192658,  -0.454648713,   0.104036709,   0.0757747856,      -0.0203675346,      0.00143495955,    -0.0000051073065, -0.00000166321949, 0.916770633,  -0.181859485,  0.0416146837,  0.0303099142, -0.00814701383,   0.00057398382, -0.0000020429226, -0.000000665287796, 0, 0, 0, 0, 0, 0, 0, 0],  # noqa
[ 10.0, 2.20500375, -0.070560004,   0.247498124,    0.0117600007,     -0.0238101257,     0.00436224566,     -0.00031423585,   0.0000081803325, 2.20500375,  -0.070560004,   0.247498124,   0.0117600007,      -0.0291767924,      0.00566224566,      -0.00042223585,   0.0000112469992, 0.802001501, -0.0282240016,  0.0989992497, 0.00470400027,  -0.0105240503,   0.00196889826,   -0.00014329434,     0.000003752133, 0, 0, 0, 0, 0, 0, 0, 0],  # noqa
[ 10.0,        3.7,            0,         -0.02,               0,   0.0000662855322, 0.000000162132615, -0.000000115425491,                 0,        2.7,           0.2,             0, -0.00133333333, -0.000000468834834,   0.00000284478193, -0.0000000241823473,                 0,         1.0,             0,             0,             0,              0,               0,                0,                  0, 0, 0, 0, 0, 0, 0, 0, 0],  # noqa
[ 10.0, 2.28385316, -0.181859485, 0.00832293673,   0.00121239657,  -0.0000271582042, -0.00000265422387, 0.0000000700228992,                 0, 3.60929743, -0.0832293673, -0.0181859485, 0.000554862449,     0.000060468368,  -0.00000103642023, -0.0000000948926948,                 0,         1.0,             0,             0,             0,              0,               0,                0,                  0, 0, 0, 0, 0, 0, 0, 0, 0],  # noqa
[ 10.0, 2.04635638,  0.151360499,  0.0130728724,  -0.00100906999,  -0.0000436819306,  0.00000204696112, 0.0000000571458752,                 0,  1.9431975,  -0.130728724,  0.0151360499, 0.000871524828,   -0.0000498586053,  -0.00000198217593,   0.000000103160937,                 0,         1.0,             0,             0,             0,              0,               0,                0,                  0, 0, 0, 0, 0, 0, 0, 0, 0],  # noqa
[ 10.0, 3.66017029, 0.0558830996, -0.0192034057, -0.000372553998,   0.0000635143987, 0.000000950551081,  -0.00000011758505,                 0,  2.4205845,   0.192034057, 0.00558830996, -0.00128022705,   -0.0000189713663,   0.00000268617272,                   0,                 0,         1.0,             0,             0,             0,              0,               0,                0,                  0, 0, 0, 0, 0, 0, 0, 0, 0],  # noqa
[ 10.0, 2.55449997, -0.197871649, 0.00291000068,   0.00131914433, -0.00000918070157, -0.00000283809877, 0.0000000407194176,                 0, 3.68935825, -0.0291000068, -0.0197871649, 0.000194000045,    0.0000656483534, -0.000000253508627,  -0.000000110678629,                 0,         1.0,             0,             0,             0,              0,               0,                0,                  0, 0, 0, 0, 0, 0, 0, 0, 0],  # noqa
[ 10.0, 1.86092847,  0.108804222,  0.0167814306, -0.000725361481,  -0.0000558733589,  0.00000141158057,  0.000000083694536,                 0, 2.15597889,  -0.167814306,  0.0108804222,  0.00111876204,   -0.0000356673429,  -0.00000247517909,  0.0000000830846231,                 0,         1.0,             0,             0,             0,              0,               0,                0,                  0, 0, 0, 0, 0, 0, 0, 0, 0],  # noqa
[ 10.0, 3.54385396,  0.107314584, -0.0168770792, -0.000715430557,    -0.00312590039,    0.000811285535,   -0.0000695256862,  0.00000201422753, 2.16342708,   0.168770792,  0.0107314584, -0.00112513861,     -0.00212050101,     0.000455868162,    -0.0000347940139, 0.000000931306555,         1.0,             0,             0,             0,        -0.0035,         0.00084,         -0.00007,           0.000002, 0, 0, 0, 0, 0, 0, 0, 0],  # noqa

]
"""
"""
figure1=[[  1.0,        2.7,            0,             0,               0,                 0,                 0,                  0,                0,        2.7,             0,             0,              0,                  0,                  0,                   0,                 0,   0, 0, 0, 0,    35.0,   -84.0,     70.0,    -20.0, 0, 0, 0, 0, 0, 0, 0, 0],  # noqa
[  1.0,        2.7,            0,             0,               0,                 0,                 0,                  0,                0,        2.7,             0,             0,              0,                  0,                  0,                   0,                 0, 1.0, 0, 0, 0,       0,       0,        0,        0, 0, 0, 0, 0, 0, 0, 0, 0],  # noqa
[  1.0,        2.7,            0,             0,               0,                 0,                 0,                  0,                0,        2.7,             0,             0,              0,                  0,                  0,                   0,                 0, 1.0, 0, 0, 0,       0,       0,        0,        0, 0, 0, 0, 0, 0, 0, 0, 0],  # noqa
[ 10.0,        2.7,            0,             0,               0,                 0,                 0,                  0,                0,        2.7,             0,             0,              0,                  0,                  0,                   0,                 0, 1.0, 0, 0, 0,       0,       0,        0,        0, 0, 0, 0, 0, 0, 0, 0, 0],  # noqa
[ 10.0,        2.7,            0,             0,               0,            0.0025,          -0.00056,           0.000044,       -0.0000012,        2.7,             0,             0,              0,     -0.00286666667,            0.00074,           -0.000064,  0.00000186666667, 1.0, 0, 0, 0,       0,       0,        0,        0, 0, 0, 0, 0, 0, 0, 0, 0],  # noqa
[ 10.0,        3.7,            0,         -0.02,               0,   0.0000662855322, 0.000000162132615, -0.000000115425491,                0,        2.7,           0.2,             0, -0.00133333333, -0.000000468834834,   0.00000284478193, -0.0000000241823473,                 0, 1.0, 0, 0, 0,       0,       0,        0,        0, 0, 0, 0, 0, 0, 0, 0, 0],  # noqa
[ 10.0, 2.28385316, -0.181859485, 0.00832293673,   0.00121239657,  -0.0000271582042, -0.00000265422387, 0.0000000700228992,                0, 3.60929743, -0.0832293673, -0.0181859485, 0.000554862449,     0.000060468368,  -0.00000103642023, -0.0000000948926948,                 0, 1.0, 0, 0, 0,       0,       0,        0,        0, 0, 0, 0, 0, 0, 0, 0, 0],  # noqa
[ 10.0, 2.04635638,  0.151360499,  0.0130728724,  -0.00100906999,  -0.0000436819306,  0.00000204696112, 0.0000000571458752,                0,  1.9431975,  -0.130728724,  0.0151360499, 0.000871524828,   -0.0000498586053,  -0.00000198217593,   0.000000103160937,                 0, 1.0, 0, 0, 0,       0,       0,        0,        0, 0, 0, 0, 0, 0, 0, 0, 0],  # noqa
[ 10.0, 3.66017029, 0.0558830996, -0.0192034057, -0.000372553998,   0.0000635143987, 0.000000950551081,  -0.00000011758505,                0,  2.4205845,   0.192034057, 0.00558830996, -0.00128022705,   -0.0000189713663,   0.00000268617272,                   0,                 0, 1.0, 0, 0, 0,       0,       0,        0,        0, 0, 0, 0, 0, 0, 0, 0, 0],  # noqa
[ 10.0, 2.55449997, -0.197871649, 0.00291000068,   0.00131914433, -0.00000918070157, -0.00000283809877, 0.0000000407194176,                0, 3.68935825, -0.0291000068, -0.0197871649, 0.000194000045,    0.0000656483534, -0.000000253508627,  -0.000000110678629,                 0, 1.0, 0, 0, 0,       0,       0,        0,        0, 0, 0, 0, 0, 0, 0, 0, 0],  # noqa
[ 10.0, 1.86092847,  0.108804222,  0.0167814306, -0.000725361481,  -0.0000558733589,  0.00000141158057,  0.000000083694536,                0, 2.15597889,  -0.167814306,  0.0108804222,  0.00111876204,   -0.0000356673429,  -0.00000247517909,  0.0000000830846231,                 0, 1.0, 0, 0, 0,       0,       0,        0,        0, 0, 0, 0, 0, 0, 0, 0, 0],  # noqa
[ 10.0, 3.54385396,  0.107314584, -0.0168770792, -0.000715430557,    -0.00312590039,    0.000811285535,   -0.0000695256862, 0.00000201422753, 2.16342708,   0.168770792,  0.0107314584, -0.00112513861,     -0.00212050101,     0.000455868162,    -0.0000347940139, 0.000000931306555, 1.0, 0, 0, 0, -0.0035, 0.00084, -0.00007, 0.000002, 0, 0, 0, 0, 0, 0, 0, 0],  # noqa
 ]"""
figure1=[[ 10.0, 2.7, 0, 0, 0, 0, 0, 0, 0, 2.7, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0,  0.0035, -0.00084,  0.00007, -0.000002, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] ,  # noqa
[ 10.0, 2.7, 0, 0, 0, 0, 0, 0, 0, 2.7, 0, 0, 0, 0, 0, 0, 0, 1.0, 0, 0, 0,       0,        0,        0,         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] ,  # noqa
[ 10.0, 2.7, 0, 0, 0, 0, 0, 0, 0, 2.7, 0, 0, 0, 0, 0, 0, 0, 1.0, 0, 0, 0,       0,        0,        0,         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] ,  # noqa
[ 10.0, 2.7, 0, 0, 0, 0, 0, 0, 0, 2.7, 0, 0, 0, 0, 0, 0, 0, 1.0, 0, 0, 0,       0,        0,        0,         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] ,  # noqa
[ 10.0, 2.7, 0, 0, 0, 0, 0, 0, 0, 2.7, 0, 0, 0, 0, 0, 0, 0, 1.0, 0, 0, 0,       0,        0,        0,         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] ,  # noqa
[ 10.0, 2.7, 0, 0, 0, 0, 0, 0, 0, 2.7, 0, 0, 0, 0, 0, 0, 0, 1.0, 0, 0, 0,       0,        0,        0,         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] ,  # noqa
[ 10.0, 2.7, 0, 0, 0, 0, 0, 0, 0, 2.7, 0, 0, 0, 0, 0, 0, 0, 1.0, 0, 0, 0,       0,        0,        0,         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] ,  # noqa
[ 10.0, 2.7, 0, 0, 0, 0, 0, 0, 0, 2.7, 0, 0, 0, 0, 0, 0, 0, 1.0, 0, 0, 0, -0.0035,  0.00084, -0.00007,  0.000002, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] ,  # noqa
]
w1 = []
w2 = []
w3 = []
w4 = []
w1bc = []
w2bc = []
w3bc = []
w4bc = []
m1 = []
m2 = []
m3 = []
f = []
c1 = []
c2 = []
c3 = []
c4 = []
x = []
y = []
z = []
dx = []
dy = []
dz = []
d2x = []
d2y = []
d2z = []
d3x = []
d3y = []
d3z = []
d4x = []
d4y = []
d4z = []
psi = []
phi=[]
theta=[]
dphi=[]
dtheta=[]
dpsi = []
d2psi = []
wx=[]
wy=[]
wz=[]
rx = []
ry = []
rz = []
rdx = []
rdy = []
rdz = []
rd2x = []
rd2y = []
rd2z = []
rd3x = []
rd3y = []
rd3z = []
rd4x = []
rd4y = []
rd4z = []
rpsi = []
rdpsi = []
rd2psi = []
t = []
stx = []
sty = []
stz = []
stdx = []
stdy = []
stdz = []
std2x = []
std2y = []
std2z = []
std3x = []
std3y = []
std3z = []
std4x = []
std4y = []
std4z = []
stpsi = []
stdpsi = []
std2psi = []
ts = []
tp=[]

tc=[]
starte=[]
startc=[]
startp=[]


class Uploader:
    def __init__(self):
        self._is_done = False

    def upload(self, trajectory_mem):
        print('Uploading data')
        trajectory_mem.write_data(self._upload_done)

        while not self._is_done:
            time.sleep(0.2)

    def _upload_done(self, mem, addr):
        print('Data uploaded')
        self._is_done = True


def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')
    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]

            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            # print("{} {} {}".
            #       format(max_x - min_x, max_y - min_y, max_z - min_z))
            # print("log")

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break


def reset_estimator(cf):
     cf.param.set_value('kalman.resetEstimation', '1')
     time.sleep(0.1)
     cf.param.set_value('kalman.resetEstimation', '0')

    #wait_for_position_estimator(cf)


def activate_high_level_commander(cf):
    cf.param.set_value('commander.enHighLevel', '1')


def activate_mellinger_controller(cf):
    cf.param.set_value('stabilizer.controller', '1')


def upload_trajectory(cf, trajectory_id, trajectory):
    trajectory_mem = cf.mem.get_mems(MemoryElement.TYPE_TRAJ)[0]

    total_duration = 0
    for row in trajectory:
        duration = row[0]
        x = Poly4D.Poly(row[1:9])
        y = Poly4D.Poly(row[9:17])
        z = Poly4D.Poly(row[17:25])
        yaw = Poly4D.Poly(row[25:33])
        trajectory_mem.poly4Ds.append(Poly4D(duration, x, y, z, yaw))
        total_duration += duration

    Uploader().upload(trajectory_mem)
    cf.high_level_commander.define_trajectory(trajectory_id, 0,
                                              len(trajectory_mem.poly4Ds))
    return total_duration

def ftime(logger, TOC1, tab1):
    for log_entry1 in logger:
        data1 = log_entry1[1]

        tab1.append(data1[TOC1])



        break
def ftraj(logger, TOC1,TOC2,TOC3, tab1,tab2,tab3):
    for log_entry1 in logger:
        data1 = log_entry1[1]

        tab1.append(data1[TOC1])
        tab2.append(data1[TOC2])
        tab3.append(data1[TOC3])


        break

def plotfig():
    fig0 = plt.figure(0)
    plt.subplot(5, 3, 1)

    plt.plot(t, rx, label='x')
    plt.plot(t, ry, label='y')
    plt.plot(t, rz, label='z')

    plt.xlabel('T (s) ')
    plt.ylabel('x,y,z (m) ')
    plt.legend()
    plt.title('References position (m)')

    plt.draw()
    plt.subplot(5, 3, 2)

    plt.plot(t, rdx, label='x')
    plt.plot(t, rdy, label='y')
    plt.plot(t, rdz, label='z')

    plt.xlabel('T (s) ')
    plt.ylabel('vx, vy, vz (m/s)')
    plt.legend()
    plt.title('References vitesses (m/s)')

    plt.draw()
    plt.subplot(5, 3, 3)

    plt.plot(t, rd2x, label='x')
    plt.plot(t, rd2y, label='y')
    plt.plot(t, rd2z, label='z')
    plt.legend()

    plt.xlabel('T (s) ')
    plt.ylabel('ax,ay,az m/s² ')
    plt.title(' References acceleration (m/s²) ')

    plt.draw()
    plt.subplot(5, 3, 4)

    plt.plot(t, x, label='x')
    plt.plot(t, y, label='y')
    plt.plot(t, z, label='z')
    plt.legend()

    plt.xlabel('T (s) ')
    plt.ylabel('x,y,z (m) ')
    plt.title('P')

    plt.draw()
    plt.subplot(5, 3, 5)

    plt.plot(t, dx, label='x')
    plt.plot(t, dy, label='y')
    plt.plot(t, dz, label='z')
    plt.legend()

    plt.xlabel('T (s) ')
    plt.ylabel('vx,vy,vz (m/s) ')
    plt.title('dP')

    plt.draw()
    plt.subplot(5, 3, 6)

    plt.plot(t, d2x, label='x')
    plt.plot(t, d2y, label='y')
    plt.plot(t, d2z, label='z')
    plt.legend()

    plt.xlabel('T (s) ')
    plt.ylabel('ax,ay,az (m/s²) ')
    plt.title('d2P')

    plt.draw()
    plt.subplot(5, 3, 7)

    plt.plot(t, stx, label='x')
    plt.plot(t, sty, label='y')
    plt.plot(t, stz, label='z')

    plt.xlabel('T (s) ')
    plt.ylabel('x,y,z (m) ')
    plt.legend()
    plt.title(' state P')

    plt.draw()
    plt.subplot(5, 3, 8)

    plt.plot(ts, stdx, label='x')
    plt.plot(ts, stdy, label='y')
    plt.plot(ts, stdz, label='z')

    plt.xlabel('T (s) ')
    plt.ylabel('vx,vy,vz (m/s) ')
    plt.legend()
    plt.title(' state dP')

    plt.draw()
    plt.subplot(5, 3, 9)

    plt.plot(ts, std2x, label='x')
    plt.plot(ts, std2y, label='y')
    plt.plot(ts, std2z, label='z')
    plt.legend()

    plt.xlabel('T (s) ')
    plt.ylabel('ax,ay,az (m/s²) ')
    plt.title(' state d2P')
    plt.draw()
    plt.subplot(5, 3, 10)

    plt.plot(t, phi, label='x')
    plt.plot(t, theta, label='y')
    plt.plot(t, psi, label='z')
    plt.legend()
    plt.title('Angles (Estimation) ')
    plt.xlabel('t')
    plt.ylabel('phi,theta,psi (rad)')
    plt.draw()

    plt.subplot(5, 3, 11)
    plt.plot(t, dphi, label='x')
    plt.plot(t, dtheta, label='y')
    plt.plot(t, dpsi, label='z')
    plt.legend()
    plt.title('dAngles/dt (Estimation) ')
    plt.xlabel('t')
    plt.ylabel('dphi/dt,dtheta/dt,dpsi/dt (rad/s)')
    plt.draw()

    plt.subplot(5, 3, 12)
    plt.plot(t, wx, label='wx')
    plt.plot(t, wy, label='wy')
    plt.plot(t, wz, label='wz')
    plt.legend()
    plt.title('Vitesses angulaire dans le body frame (Estimation) ')
    plt.xlabel('t')
    plt.ylabel('wx  wy wz (rad/s)')
    plt.draw()

    plt.subplot(5, 3, 13)

    plt.plot(tc, startc)
    plt.title('Tic start commande')
    plt.xlabel('t')
    plt.ylabel('1/0')
    plt.draw()

    plt.subplot(5, 3, 14)
    plt.plot(tp, startp)
    plt.title('Tic start powerDistribution')
    plt.xlabel('t')
    plt.ylabel('1/0')
    plt.draw()

    plt.subplot(5, 3, 15)
    plt.plot(t, starte)
    plt.title('Tic start estimator')
    plt.xlabel('t')
    plt.ylabel('1/0')
    plt.draw()


    plt.pause(0.05)

    plt.figure(2)
    plt.subplot(1, 4, 1)

    plt.plot(tc, c1, label='c1 (x)')
    plt.plot(tc, c2, label='c2 (y) ')
    plt.plot(tc, c3, label='c3 (z) ')
    plt.plot(tc, c4, label='c4 (psi) ')

    plt.xlabel('T (s) ')
    plt.ylabel('Commande [d4p;d2psi] ')
    plt.legend()
    plt.title(' Commande V=[d4p;d2psi]-K(X-Xref)')
    plt.draw()

    plt.subplot(1, 4, 3)

    plt.plot(tp, m1, label='m1 (x)')
    plt.plot(tp, m2, label='m2 (y) ')
    plt.plot(tp, m3, label='m3 (z) ')

    plt.xlabel('T (s) ')
    plt.ylabel('[M] N.m ')
    plt.legend()
    plt.title(' Moments N.m ')
    # plt.pause(Periode / 1000)
    plt.draw()
    plt.subplot(1, 4, 4)

    plt.plot(tp, f, label='f  (m/s²) ')

    plt.xlabel('T (s) ')
    plt.ylabel('[f] m/s² ')
    plt.legend()
    plt.title(' force m/s² ')
    # plt.pause(Periode / 1000)
    plt.draw()

    plt.subplot(1, 4, 2)

    plt.plot(tp, w1, label='w1')
    plt.plot(tp, w2, label='w2')
    plt.plot(tp, w3, label='w3')
    plt.plot(tp, w4, label='w4')

    plt.xlabel('T (s) ')
    plt.ylabel('Rpm')
    plt.legend()
    plt.title(' References moteurs ENSEM')

    plt.draw()

    plt.pause(0.01)
    plt.pause(10000)
"""
    plt.figure(3)
    plt.plot(tp,w1bc,label='w1_BC')
    plt.plot(tp,w2bc,label='w1_BC')
    plt.plot(tp,w3bc,label='w1_BC')
    plt.plot(tp,w4bc,label='w1_BC')
    plt.xlabel('T (s) ')
    plt.ylabel('Rpm')
    plt.legend()
    plt.title(' References moteurs BC')
    plt.draw()
    plt.pause(10000)
    plt.clf()
"""

def run_sequence(cf, trajectory_id, duration):

    activate_high_level_commander(cf)
    commander = cf.high_level_commander

    cf.param.set_value('stabilizer.isInit', '0')
    cf.param.set_value('stabilizer.controller', '2')
    cf.param.set_value('stabilizer.estimator', '2')
    while (cf.param.is_updated.__bool__() == False):
        time.sleep(0.01)
        print(cf.param.is_updated.__bool__())



    relative = False


    commander.start_trajectory(trajectory_id, 1.0, relative)

    #commander.go_to(2.5,2.5,2,0,1,relative)
    print('switch')
    print('takeoff')
    Periode = 180
    log_config1 = LogConfig(name='LOG1', period_in_ms=Periode)
    log_config2 = LogConfig(name='LOG2', period_in_ms=Periode)
    log_config3 = LogConfig(name='LOG3', period_in_ms=Periode)
    log_config4 = LogConfig(name='LOG4', period_in_ms=Periode)
    log_config5 = LogConfig(name='LOG4', period_in_ms=Periode)
    log_config6 = LogConfig(name='LOG4', period_in_ms=Periode)

    log_config7 = LogConfig(name='LOG7', period_in_ms=Periode)
    log_config8 = LogConfig(name='LOG8', period_in_ms=Periode)
    log_config9 = LogConfig(name='LOG9', period_in_ms=Periode)
    log_config10 = LogConfig(name='LOG10', period_in_ms=Periode)

    log_config13 = LogConfig(name='LOG13', period_in_ms=Periode)
    log_config14 = LogConfig(name='LOG14', period_in_ms=Periode)
    log_config15 = LogConfig(name='LOG15', period_in_ms=Periode)
    log_config16 = LogConfig(name='LOG16', period_in_ms=Periode)
    log_config17 = LogConfig(name='LOG17', period_in_ms=Periode)
    log_config18 = LogConfig(name='LOG18', period_in_ms=Periode)



    log_config1.add_variable('Xlog.phi', 'float')
    log_config1.add_variable('Xlog.theta', 'float')
    log_config1.add_variable('Xlog.psi', 'float')
    log_config2.add_variable('Xlog.dphi', 'float')
    log_config2.add_variable('Xlog.dtheta', 'float')
    log_config2.add_variable('Xlog.dpsi', 'float')
    log_config3.add_variable('Xlog.w1', 'float')
    log_config3.add_variable('Xlog.w2', 'float')
    log_config3.add_variable('Xlog.w3', 'float')
    log_config4.add_variable('Refflog.x1', 'float')
    log_config4.add_variable('Refflog.x2', 'float')
    log_config4.add_variable('Refflog.x3', 'float')
    log_config5.add_variable('Refflog.x4', 'float')
    log_config5.add_variable('Refflog.x5', 'float')
    log_config5.add_variable('Refflog.x6', 'float')
    log_config6.add_variable('Refflog.x7', 'float')
    log_config6.add_variable('Refflog.x8', 'float')
    log_config6.add_variable('Refflog.x9', 'float')
    log_config7.add_variable('Xlog.x1', 'float')
    log_config7.add_variable('Xlog.x2', 'float')
    log_config7.add_variable('Xlog.x3', 'float')
    log_config7.add_variable('Xlog.t', 'float')
    log_config7.add_variable('Xlog.starte', 'int16_t')


    log_config8.add_variable('Xlog.x4', 'float')
    log_config8.add_variable('Xlog.x5', 'float')
    log_config8.add_variable('Xlog.x6', 'float')
    log_config9.add_variable('Xlog.x7', 'float')
    log_config9.add_variable('Xlog.x8', 'float')
    log_config9.add_variable('Xlog.x9', 'float')
    log_config10.add_variable('Xlog.x10', 'float')
    log_config10.add_variable('Xlog.x11', 'float')
    log_config10.add_variable('Xlog.x12', 'float')


    log_config13.add_variable('Commandelog.c1', 'float')
    log_config13.add_variable('Commandelog.c2', 'float')
    log_config13.add_variable('Commandelog.c3', 'float')
    log_config13.add_variable('Commandelog.c4', 'float')
    log_config13.add_variable('Commandelog.startc', 'int16_t')
    log_config13.add_variable('Commandelog.tc', 'float')

    log_config14.add_variable('Powerlog.m1', 'float')
    log_config14.add_variable('Powerlog.m2', 'float')
    log_config14.add_variable('Powerlog.m3', 'float')
    log_config14.add_variable('Powerlog.f', 'float')
    log_config15.add_variable('Powerlog.w1', 'float')
    log_config15.add_variable('Powerlog.w2', 'float')
    log_config15.add_variable('Powerlog.w3', 'float')
    log_config15.add_variable('Powerlog.w4', 'float')
    log_config15.add_variable('Powerlog.startp', 'int16_t')
    log_config15.add_variable('Powerlog.tp', 'float')

    log_config16.add_variable('STlog.x1', 'float')
    log_config16.add_variable('STlog.x2', 'float')
    log_config16.add_variable('STlog.x3', 'float')
    log_config16.add_variable('STlog.t', 'float')
    log_config16.add_variable('STlog.starte', 'int16_t')


    log_config17.add_variable('STlog.x4', 'float')
    log_config17.add_variable('STlog.x5', 'float')
    log_config17.add_variable('STlog.x6', 'float')
    log_config18.add_variable('STlog.x7', 'float')
    log_config18.add_variable('STlog.x8', 'float')
    log_config18.add_variable('STlog.x9', 'float')


    logger1 = SyncLogger(cf, log_config1)
    logger2 = SyncLogger(cf, log_config2)
    logger3 = SyncLogger(cf, log_config3)
    logger4 = SyncLogger(cf, log_config4)
    logger5 = SyncLogger(cf, log_config5)
    logger6 = SyncLogger(cf, log_config6)

    logger7 = SyncLogger(cf, log_config7)
    logger8 = SyncLogger(cf, log_config8)
    logger9 = SyncLogger(cf, log_config9)
    logger10 = SyncLogger(cf, log_config10)

    logger13 = SyncLogger(cf, log_config13)
    logger14 = SyncLogger(cf, log_config14)
    logger15 = SyncLogger(cf, log_config15)

    logger16 = SyncLogger(cf, log_config16)
    logger17 = SyncLogger(cf, log_config17)
    logger18 = SyncLogger(cf, log_config18)




    logger1.connect()
    logger2.connect()
    logger3.connect()
    logger4.connect()
    logger5.connect()
    logger6.connect()
    logger7.connect()
    logger8.connect()
    logger9.connect()
    logger10.connect()

    logger13.connect()
    logger14.connect()
    logger15.connect()
    logger16.connect()
    logger17.connect()
    logger18.connect()

    time.sleep(1)

    active=False
    active2=False
    t0= time.time()

    while (time.time() <t0+30):



        for log_entry1 in logger16:
            data1 = log_entry1[1]

            stx.append(data1['STlog.x1'])
            sty.append(data1['STlog.x2'])
            stz.append(data1['STlog.x3'])
            ts.append(data1['STlog.t'])

            break
        for log_entry1 in logger7:
            data1 = log_entry1[1]

            x.append(data1['Xlog.x1'])
            y.append(data1['Xlog.x2'])
            z.append(data1['Xlog.x3'])
            t.append(data1['Xlog.t'])
            starte.append(data1['Xlog.starte'])

            break
        ftraj(logger8,'Xlog.x4','Xlog.x5','Xlog.x6',dx,dy,dz)
        ftraj(logger9,'Xlog.x7','Xlog.x8','Xlog.x9',d2x,d2y,d2z)
        ftraj(logger10,'Xlog.x10','Xlog.x11','Xlog.x12',d3x,d3y,d3z)
        ftraj(logger1,'Xlog.phi','Xlog.theta','Xlog.psi',phi,theta,psi)
        ftraj(logger2,'Xlog.dphi','Xlog.dtheta','Xlog.dpsi',dphi,dtheta,dpsi)
        ftraj(logger3,'Xlog.w1','Xlog.w2','Xlog.w3',wx,wy,wz)
        ftraj(logger4,'Refflog.x1','Refflog.x2','Refflog.x3',rx,ry,rz)
        ftraj(logger5,'Refflog.x4','Refflog.x5','Refflog.x6',rdx,rdy,rdz)
        ftraj(logger6,'Refflog.x7','Refflog.x8','Refflog.x9',rd2x,rd2y,rd2z)

        ftraj(logger17,'STlog.x4','STlog.x5','STlog.x6',stdx,stdy,stdz)
        ftraj(logger18,'STlog.x7','STlog.x8','STlog.x9',std2x,std2y,std2z)

        for log_entry1 in logger13:
            data1 = log_entry1[1]

            c1.append(data1['Commandelog.c1'])
            c2.append(data1['Commandelog.c2'])
            c3.append(data1['Commandelog.c3'])
            c4.append(data1['Commandelog.c4'])
            startc.append(data1['Commandelog.startc'])
            tc.append(data1['Commandelog.tc'])
            break
        for log_entry1 in logger14:
            data1 = log_entry1[1]

            m1.append(data1['Powerlog.m1'])
            m2.append(data1['Powerlog.m2'])
            m3.append(data1['Powerlog.m3'])
            f.append(data1['Powerlog.f'])
            break
        for log_entry1 in logger15:
            data1 = log_entry1[1]

            w1.append(data1['Powerlog.w1'])
            w2.append(data1['Powerlog.w2'])
            w3.append(data1['Powerlog.w3'])
            w4.append(data1['Powerlog.w4'])
            startp.append(data1['Powerlog.startp'])
            tp.append(data1['Powerlog.tp'])
            break




        """ 
        plt.figure(1)
        plt.subplot(2, 1, 1)
        plt.plot(rx, ry)
        plt.title('ReffX ReffY')
        plt.xlabel('Xreff (m)')
        plt.ylabel('Yreff (m) ')
        plt.draw()
        plt.subplot(2, 1, 2)
        plt.plot(x, y)
        plt.title('X Y')
        plt.xlabel('X (m)')
        plt.ylabel('Y (m) ')
        plt.draw()
        

        plt.pause(0.0001)
        plt.clf()
        """




        print(tp[len(tp)-1]-tp[0])
        if(tp[len(tp)-1]>=tp[0]+25 and active==True and active2==False):
            cf.param.set_value('stabilizer.isInit', '1')
            active2=True
            print('go')
        if(active==False and tp[len(tp)-1]>=tp[0]+15):
            active=True


            cf.param.set_value('stabilizer.controller', '4')
            cf.param.set_value('stabilizer.estimator', '3')
            print('Start')
            #time.sleep(3)



            while (cf.param.is_updated.__bool__() == False):
                time.sleep(0.1)
                print(cf.param.is_updated.__bool__())

    plotfig()
    time.sleep(20)
    cf.param.set_value('stabilizer.controller', '2')
    cf.param.set_value('stabilizer.estimator', '2')
    time.sleep(5)

    commander.land(0.0, 2.0)
    time.sleep(2)
    commander.stop()


if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf

        reset_estimator(cf)
        trajectory_id = 1
        duration = upload_trajectory(cf, trajectory_id, figure1)


        print('The sequence is {:.1f} seconds long'.format(duration))
        #reset_estimator(cf)
        run_sequence(cf, trajectory_id, duration)

